#include "common.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/StreamComposite.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/OpStatus.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <signal.h>
#include <thread>
#include "kiss_fft.h"
#include <condition_variable>
#include <mutex>
#include <filesystem>
#include "args.hxx"

#include "../src/DSP/FFT/FFT.h"
#include "../src/DSP/math/math.h"

#define USE_GNU_PLOT 1
#ifdef USE_GNU_PLOT
    #include "gnuPlotPipe.h"
#endif

using namespace lime;
using namespace std;

std::mutex globalGnuPlotMutex; // Seems multiple plot pipes can't be used concurrently
bool showPlots = false;

bool stopProgram(false);
void intHandler(int dummy)
{
    //std::cerr << "Stopping\n"sv;
    stopProgram = true;
}

static LogLevel logVerbosity = LogLevel::Error;
static LogLevel strToLogLevel(const std::string_view str)
{
    if ("debug"sv == str)
        return LogLevel::Debug;
    else if ("verbose"sv == str)
        return LogLevel::Verbose;
    else if ("error"sv == str)
        return LogLevel::Error;
    else if ("warning"sv == str)
        return LogLevel::Warning;
    else if ("info"sv == str)
        return LogLevel::Info;
    return LogLevel::Error;
}

static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    cerr << msg << endl;
}

static std::vector<int> ParseIntArray(args::NargsValueFlag<int>& flag)
{
    std::vector<int> numbers;
    for (const auto& number : args::get(flag))
        numbers.push_back(number);
    return numbers;
}

static lime::complex32f_t Chirp(double w1, double w2, double amplitude, double chirpDuration, double time)
{
    double phase = w1 * time + (w2 - w1) * time * time / (2 * chirpDuration);
    return lime::complex32f_t(amplitude * cos(phase), amplitude * sin(phase));
}

static std::vector<lime::complex32f_t> GenerateChirp(double duration, double sampleRate, double w1, double w2)
{
    std::vector<lime::complex32f_t> values;
    double step = 1.0 / sampleRate;
    double angularF = w1 * 2 * M_PI * sampleRate / 2;
    double angularF2 = w2 * 2 * M_PI * sampleRate / 2;
    double amplitude = 0.7;
    for (double t = 0; t < duration; t += step)
    {
        auto v = Chirp(angularF, angularF2, amplitude, duration, t);
        values.push_back(v);
    }
    return values;
}

static void PlotSamples(std::vector<complex32f_t>& samples)
{
    if (!showPlots)
        return;

    GNUPlotPipe plot;
    //plot.writef("set yrange[%f:%f]\n", -1.0, 1.0);
    plot.write("plot '-' with lines, '-' with lines\n");
    uint32_t i = 0;
    for (const auto& s : samples)
        plot.writef("%i %f\n", i++, s.real());
    plot.write("e\n");
    i = 0;
    for (const auto& s : samples)
        plot.writef("%i %f\n", i++, s.imag());
    plot.write("e\n");
    plot.flush();
}

static void PlotSamples(const complex32f_t* samples, size_t count)
{
    if (!showPlots)
        return;

    GNUPlotPipe plot;
    plot.writef("set yrange[%f:%f]\n", -1.0, 1.0);
    plot.write("plot '-' with lines, '-' with lines\n");
    uint32_t i = 0;
    for (; i < count; ++i)
        plot.writef("%i %f\n", i, samples[i].real());
    plot.write("e\n");
    i = 0;
    for (; i < count; ++i)
        plot.writef("%i %f\n", i, samples[i].imag());
    plot.write("e\n");
    plot.flush();
}

static void Plotline(std::vector<double>& samples)
{
    if (!showPlots)
        return;

    GNUPlotPipe plot;
    //plot.writef("set yrange[%f:%f]\n", -1.0, 1.0);
    plot.write("plot '-' with lines, '-' with lines\n");
    uint32_t i = 0;
    for (const auto& s : samples)
        plot.writef("%i %f\n", i++, s);
    plot.write("e\n");
    plot.flush();
}

template<class T> size_t GetMaxElementIndex(const std::vector<T>& values)
{
    size_t maxIndex = 0;
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (values[i] > values[maxIndex])
            maxIndex = i;
    }
    return maxIndex;
}

#include <cassert>
static std::vector<int> SyncSamples(
    const complex32f_t* const* samples, const size_t chCount, size_t count, const std::vector<complex32f_t>& refSignal)
{
    std::vector<int> offsets;

    std::vector<complex32f_t> inputs(count);
    for (uint8_t c = 0; c < chCount; ++c)
    {
        memcpy(inputs.data(), samples[c], sizeof(complex32f_t) * inputs.size());
        auto correlation = CrossCorrelation(inputs, refSignal);
        int ci = GetMaxElementIndex(correlation);
        // printf("Ch%i correlation: %f\n", c, correlation[ci]);
        offsets.push_back(ci);
        //Plotline(correlation);
    }

    return offsets;
}

OpStatus MeasureChannelDelays(SDRDevice* device,
    StreamComposite* composite,
    bool useComposite,
    const std::vector<complex32f_t>& chirp,
    uint8_t channelCount,
    double sampleRate,
    uint8_t transmitChannel,
    std::vector<int>& sampleOffset)
{
    sampleOffset.clear();
    OpStatus status = OpStatus::Success;
    StreamMeta txMeta{};
    txMeta.waitForTimestamp = true;
    txMeta.timestamp = sampleRate / 100; // send tx samples 100ms after start
    txMeta.flushPartialPacket = true;
    const int chipIndex = 0;

    const uint32_t toSend = chirp.size();

    std::vector<const complex32f_t*> txSamples;
    std::vector<complex32f_t> nulldata(toSend);
    for (int i = 0; i < channelCount; ++i)
        txSamples.push_back(chirp.data());
    //txSamples[transmitChannel] = chirp.data();

    // receive the expected chirp
    const size_t rxSize = toSend * 2;
    std::vector<complex32f_t> rxChannel[16];
    complex32f_t* rxBuffers[16];
    for (int i = 0; i < channelCount; ++i)
    {
        rxChannel[i].resize(rxSize);
        rxBuffers[i] = rxChannel[i].data();
    }

    if (useComposite)
        composite->StreamStart();
    else
        device->StreamStart(chipIndex);

    // send chirp
    uint32_t samplesSent = useComposite ? composite->StreamTx(txSamples.data(), toSend, &txMeta)
                                        : device->StreamTx(chipIndex, txSamples.data(), toSend, &txMeta);
    if (samplesSent != toSend)
        return OpStatus::IOFailure;

    StreamMeta rxMeta{};
    const int64_t burstStart = txMeta.timestamp - toSend / 2; // approximate point of the expected data
    int samplesToSkip = burstStart;

    // skip all samples until the expected time
    while (samplesToSkip > 0)
    {
        auto timeout = std::chrono::microseconds(1000000);
        const uint32_t toRead = samplesToSkip > rxSize ? rxSize : samplesToSkip;
        const uint32_t samplesRead = useComposite ? composite->StreamRx(rxBuffers, toRead, &rxMeta, timeout)
                                                  : device->StreamRx(chipIndex, rxBuffers, toRead, &rxMeta, timeout);
        if (samplesRead != toRead)
        {
            status = OpStatus::IOFailure;
            break;
        }
        samplesToSkip -= samplesRead;
    }

    const uint32_t samplesRead =
        useComposite ? composite->StreamRx(rxBuffers, rxSize, &rxMeta) : device->StreamRx(chipIndex, rxBuffers, rxSize, &rxMeta);
    if (samplesRead != rxSize)
        status = OpStatus::IOFailure;

    if (useComposite)
    {
        composite->StreamStop();
        // composite->StreamDestroy();
    }
    else
    {
        device->StreamStop(chipIndex);
        // device->StreamDestroy(chipIndex);
    }

    for (int c = 0; c < channelCount; ++c)
        PlotSamples(rxBuffers[c], rxSize);

    sampleOffset = SyncSamples(rxBuffers, channelCount, rxSize, chirp);
    // printf("Samples offsets:\n");
    // for (const auto& v : sampleOffset)
    //     printf("sent Tx:%li , detected Rx:%li, diff:%li, index:%i\n",
    //         txMeta.timestamp,
    //         rxMeta.timestamp + v,
    //         rxMeta.timestamp + v - txMeta.timestamp,
    //         v);
    return status;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser                parser("limeSync - sync channels timing", "");
    args::HelpFlag                      helpFlag(parser, "help", "This help", {'h', "help"});

    args::ValueFlag<std::string>        deviceFlag(parser, "name", "Specifies which device to use", {'d', "device"});
    args::NargsValueFlag<int>           chipFlag(parser, "index", "Specify chip index, or index list for aggregation [0,1...]", {'c', "chip"}, args::Nargs{1, static_cast<size_t>(-1)}); // Arg count range [1, size_t::maxValue]

    args::ValueFlag<int64_t>            samplesCountFlag(parser, "sample count", "Number of samples to receive", {'s', "samplesCount"}, 0, args::Options{});
    args::ValueFlag<int64_t>            timeFlag(parser, "ms", "Time duration in milliseconds to receive", {"time"}, 0, args::Options{});

    args::ValueFlag<std::string>        logFlag(parser, "", "Log verbosity: info, warning, error, verbose, debug", {'l', "log"}, "error", args::Options{});
    args::ImplicitValueFlag<int>        mimoFlag(parser, "channel count", "use multiple channels", {"mimo"}, 2, args::Options{});
    args::Flag plotsFlag(parser, "plot", "Draw information plots", { 'p', "plot" });
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (args::Help&)
    {
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }
    showPlots = plotsFlag;

    const std::string devName = args::get(deviceFlag);
    const int channelCount = mimoFlag ? args::get(mimoFlag) : 1;

    std::vector<int> chipIndexes = ParseIntArray(chipFlag);

    StreamComposite* composite = nullptr;
    logVerbosity = strToLogLevel(args::get(logFlag));
    int chipIndex = 0;
    bool useComposite = false;

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    device->SetMessageLogCallback(LogCallback);
    lime::registerLogHandler(LogCallback);

    // if chip index is not specified and device has only one, use it by default
    if (chipIndexes.empty() && device->GetDescriptor().rfSOC.size() == 1)
        chipIndexes.push_back(0);

    StreamConfig stream;
    try
    {
        // Samples data streaming configuration
        for (int i = 0; i < channelCount; ++i)
        {
            stream.channels.at(TRXDir::Rx).push_back(i);
            stream.channels.at(TRXDir::Tx).push_back(i);
        }

        stream.format = DataFormat::F32;
        stream.linkFormat = DataFormat::I12;

        useComposite = chipIndexes.size() > 1;
        if (useComposite)
        {
            std::vector<StreamAggregate> aggregates(chipIndexes.size());
            for (size_t i = 0; i < chipIndexes.size(); ++i)
            {
                aggregates[i].device = device;
                aggregates[i].streamIndex = chipIndexes[i];
                int deviceChannelCount = device->GetDescriptor().rfSOC[chipIndexes[i]].channelCount;
                for (int j = 0; j < deviceChannelCount; ++j)
                    aggregates[i].channels.push_back(j);
            }
            composite = new StreamComposite(std::move(aggregates));
            //composite->StreamSetup(stream);
        }
        else
        {
            chipIndex = chipIndexes.empty() ? 0 : chipIndexes[0];
            // OpStatus status = device->StreamSetup(stream, chipIndex);
            // if (status != OpStatus::Success)
            // {
            //     cerr << "Failed to setup data stream.\n";
            //     return EXIT_FAILURE;
            // }
        }
    } catch (std::runtime_error& e)
    {
        std::cout << "Failed to configure settings: "sv << e.what() << std::endl;
        return -1;
    } catch (std::logic_error& e)
    {
        std::cout << "Failed to configure settings: "sv << e.what() << std::endl;
        return -1;
    }

    float sampleRate = device->GetSampleRate(chipIndex, TRXDir::Rx, 0);
    if (sampleRate <= 0)
        sampleRate = 1; // sample rate read-back not available, assign default value

    int chirp_len = 1360 / 2;
    double fs = 1e6;
    double chirpTime = chirp_len / fs;
    auto chirp = GenerateChirp(chirpTime, fs, 0.005, 0.04);
    //PlotSamples(chirpSamples);
    // std::vector<float> window;
    // FFT::GenerateWindowCoefficients(FFT::WindowFunctionType::HANNING, chirp_len + 1, window);
    // for (size_t i=0; i<chirp.size(); ++i)
    // {
    //     chirp[i].real(chirp[i].real() * window[i]);
    //     chirp[i].imag(chirp[i].imag() * window[i]);
    // }
    //PlotSamples(chirp);

    printf("    ");
    for (int i = 0; i < channelCount; ++i)
        printf("\t| Rx%i  ", i);
    printf("\n");

    for (int TxIndex = 0; TxIndex < channelCount; ++TxIndex)
    {
        if (useComposite)
            composite->StreamSetup(stream);
        else
            device->StreamSetup(stream, chipIndex);

        std::vector<int> sampleOffsets;
        OpStatus ret =
            MeasureChannelDelays(device, composite, useComposite, chirp, channelCount, sampleRate, TxIndex, sampleOffsets);
        if (ret != OpStatus::Success)
            break;

        if (useComposite)
            composite->StreamDestroy();
        else
            device->StreamDestroy(chipIndex);

        printf("Tx%i ", TxIndex);
        for (const auto& v : sampleOffsets)
            printf("\t %4i", v);
        printf("\n");
    }

    if (composite)
        delete composite;
    DeviceRegistry::freeDevice(device);
    return 0;
}
