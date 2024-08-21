/*
 * LimeMicroSystem transceiver driver
 * Copyright (C) 2015-2018 Amarisoft/LimeMicroSystems
 */
#include "limesuiteng/LimePlugin.h"
#include "limesuiteng/StreamConfig.h"

#include <stdarg.h>

extern "C" {
#include "include/trx_driver.h"
}

using namespace std;
using namespace lime;

class AmarisoftParamProvider : public LimeSettingsProvider
{
  public:
    AmarisoftParamProvider() {}

    void Init(TRXState* s) { state = s; }

    void Block() { blockAccess = true; }

    bool GetString(std::string& dest, const char* varname) override
    {
        if (blockAccess)
        {

            return false;
        }
        char* ctemp = trx_get_param_string(state, varname);
        if (!ctemp)
            return false;

        dest = std::string(ctemp);
        free(ctemp);
        return true;
    }

    bool GetDouble(double& dest, const char* varname) override
    {
        if (blockAccess)
        {
            return false;
        }
        return trx_get_param_double(state, &dest, varname) == 0 ? true : false;
    }

  private:
    TRXState* state{ nullptr };
    bool blockAccess{ false };
};

static lime::LogLevel logVerbosity = lime::LogLevel::Debug;
static void Log [[gnu::format(printf, 2, 3)]] (LogLevel lvl, const char* format, ...)
{
    if (lvl > logVerbosity)
        return;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}

static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    printf("%s\n", msg.c_str());
}

//static TRXStatistics trxstats;
static int limesuiteng_trx_get_stats(TRXState* s, TRXStatistics* m)
{
    //LimePluginContext* lime = (LimePluginContext*)s->opaque;
    // TODO:
    // for (long p = 0; p < lime->deviceCount; ++p)
    // {
    //     StreamStatus& stats = portStreamStates[p];
    //     trxstats.rx_overflow_count = stats.rx.overrun + stats.rx.loss;
    //     trxstats.tx_underflow_count = stats.tx.late + stats.tx.underrun;
    // }
    // memcpy(m, &trxstats, sizeof(TRXStatistics));
    return 0;
}

/* Callback must allocate info buffer that will be displayed */
static void limesuiteng_trx_dump_info(TRXState* s, trx_printf_cb cb, void* opaque)
{
    // LimePluginContext* lime = (LimePluginContext*)s->opaque;
    // std::stringstream ss;
    /*    for (long p = 0; p < lime->deviceCount; ++p)
    {
        SDRDevice* dev = lime->device[p];
        StreamStatus& stats = portStreamStates[p];

        StreamStats rx;
        StreamStats tx;
        dev->StreamStatus(lime->chipIndex[p], &rx, &tx);
        stats.rx.dataRate_Bps = rx.dataRate_Bps;
        stats.tx.dataRate_Bps = tx.dataRate_Bps;

        ss << "\nPort" << p << " [Rx| Loss: " << stats.rx.loss << " overrun: " << stats.rx.overrun
           << " rate: " << stats.rx.dataRate_Bps / 1e6 << " MB/s]"
           << "[Tx| Late: " << stats.tx.late << " underrun: " << stats.tx.underrun << " rate: " << stats.tx.dataRate_Bps / 1e6
           << " MB/s]"
           << " linkFormat: " << (lime->linkFormat[p] == DataFormat::I16 ? "I16" : "I12") << std::endl;
        // TODO: read FIFO usage
    }*/
    // const int len = ss.str().length();
    // char* buffer = (char*)malloc(len + 1);
    // cb(buffer, "%s\n", ss.str().c_str());
}

static void limesuiteng_trx_write_func(
    TRXState* s, trx_timestamp_t timestamp, const void** samples, int count, int flags, int tx_port_index)
{
    if (!samples) // Nothing to transmit
        return;

    StreamMeta meta{};
    meta.timestamp = timestamp;
    meta.waitForTimestamp = true;
    meta.flushPartialPacket = (flags & TRX_WRITE_MD_FLAG_END_OF_BURST);

    // samples format conversion is done internally
    LimePluginContext* lime = static_cast<LimePluginContext*>(s->opaque);
    LimePlugin_Write_complex32f(lime, reinterpret_cast<const lime::complex32f_t* const*>(samples), count, tx_port_index, meta);
}

// count - can be configured by 'trx_get_tx_samples_per_packet_func' callback
// if samples==NULL, Tx has to be disabled
// samples are ranged [-1.0 : +1.0]
static void limesuiteng_trx_write_func2(
    TRXState* s, trx_timestamp_t timestamp, const void** samples, int count, int port, TRXWriteMetadata* md)
{
    if (!samples) // Nothing to transmit
        return;

    StreamMeta meta{};
    meta.timestamp = timestamp;
    meta.waitForTimestamp = true;
    meta.flushPartialPacket = (md->flags & TRX_WRITE_MD_FLAG_END_OF_BURST);

    // samples format conversion is done internally
    LimePluginContext* lime = static_cast<LimePluginContext*>(s->opaque);
    // if (md->flags & TRX_WRITE_MD_FLAG_CUR_TIMESTAMP_REQ)
    // {
    //     md->cur_timestamp = lime->ports[port].composite->GetHardwareTimestamp();
    //     md->cur_timestamp_set = 1;
    // }
    LimePlugin_Write_complex32f(lime, reinterpret_cast<const lime::complex32f_t* const*>(samples), count, port, meta);
}

static int limesuiteng_trx_read_func(TRXState* s, trx_timestamp_t* ptimestamp, void** samples, int count, int rx_port_index)
{
    StreamMeta meta{};
    meta.waitForTimestamp = false;
    meta.flushPartialPacket = false;

    LimePluginContext* lime = static_cast<LimePluginContext*>(s->opaque);
    int samplesGot = LimePlugin_Read_complex32f(lime, reinterpret_cast<lime::complex32f_t**>(samples), count, rx_port_index, meta);
    *ptimestamp = meta.timestamp; // if timestamp is not updated, amarisoft will freeze
    return samplesGot;
}

// Read has to block until at least 1 sample is available and must return at most 'count' samples
// timestamp is samples counter
// return number of samples produced
static int limesuiteng_trx_read_func2(
    TRXState* s, trx_timestamp_t* ptimestamp, void** samples, int count, int port, TRXReadMetadata* md)
{
    StreamMeta meta{};
    meta.waitForTimestamp = false;
    meta.flushPartialPacket = false;
    md->flags = 0;

    LimePluginContext* lime = static_cast<LimePluginContext*>(s->opaque);
    int samplesGot = LimePlugin_Read_complex32f(lime, reinterpret_cast<lime::complex32f_t**>(samples), count, port, meta);
    *ptimestamp = meta.timestamp; // if timestamp is not updated, amarisoft will freeze
    return samplesGot;
}

// Return in *psample_rate the sample rate supported by the device
// corresponding to a LTE bandwidth of 'bandwidth' Hz. Also return
// in n=*psample_rate_num the wanted sample rate before the
// interpolator as 'n * 1.92' MHz. 'n' must currently be of the
// form 2^n1*3^n2*5^n3.
// Return 0 if OK, -1 if none.
static int limesuiteng_trx_get_sample_rate_func(TRXState* s1, TRXFraction* psample_rate, int* psample_rate_num, int bandwidth)
{
    LimePluginContext* s = static_cast<LimePluginContext*>(s1->opaque);
    // multipliers that can be made using 2^n1*3^n2*5^n3, n1 >= 1
    const uint8_t multipliers[] = { 2,
        4,
        6,
        8,
        10,
        12,
        16,
        18,
        20,
        24,
        //30, 32, 36, 40, 48, 50, 54, 60, 64
        // 8, 10, 12, 16, 24,
        32,
        40,
        48,
        50,
        64 };

    // limesuiteng_trx_get_sample_rate_func seems to be called for each Port, but the API does not provide index.
    // workaround here assume that they are being configured in order 0,1,2...
    static int whichPort = 0;
    int p = whichPort;
    DevNode* node = s->ports[p].nodes[0];
    if (!node || !node->device)
    {
        Log(LogLevel::Error, "%s: No devices are assigned to port%i", __func__, p);
        return -1;
    }
    double& rate = node->config.channel[0].rx.sampleRate;
    whichPort = (whichPort + 1); // mod, just in case.

    const float desiredSamplingRate = bandwidth * 1.536;
    //printf ("Required bandwidth:[%u], current sample_rate [%u]", bandwidth, s->sample_rate);
    if (rate <= 0) // sample rate not specified, align on 1.92Mhz
    {
        Log(LogLevel::Verbose,
            "Port[%i] Trying sample rates which are bandwidth(%u) * 1.536 = %f",
            p,
            bandwidth,
            desiredSamplingRate);
        for (uint32_t i = 0; i < sizeof(multipliers) / sizeof(uint8_t); ++i)
        {
            int n = multipliers[i];
            if (n * 1920000 > 122.88e6)
                break;
            Log(LogLevel::Debug, "\tPort[%i] Trying sample rate : bandwidth(%u) sample_rate(%u)", p, bandwidth, n * 1920000);
            if (desiredSamplingRate <= n * 1920000)
            {
                *psample_rate_num = n;
                psample_rate->num = n * 1920000;
                psample_rate->den = 1;
                rate = psample_rate->num;
                Log(LogLevel::Info, "Port[%i] Automatic sample rate: %g MSps, n = [%u] * 1.92e6", p, rate / 1e6, n);
                return 0;
            }
        }
        Log(LogLevel::Verbose, "Port[%i] Trying sample rates which are close to bandwidth(%u)", p, bandwidth);
        for (uint32_t i = 0; i < sizeof(multipliers) / sizeof(uint8_t); ++i)
        {
            int n = multipliers[i];
            Log(LogLevel::Debug, "\tPort[%i] Trying sample rate : bandwidth(%u) sample_rate(%u)", p, bandwidth, n * 1920000);
            if (bandwidth <= n * 1920000)
            {
                *psample_rate_num = n;
                psample_rate->num = n * 1920000;
                psample_rate->den = 1;
                rate = psample_rate->num;
                Log(LogLevel::Info, "Port[%i] Automatic sample rate: %g MSps, n = [%u] * 1.92e6", p, rate / 1e6, n);
                return 0;
            }
        }
        Log(LogLevel::Error, "Port[%i] Could not find suitable sampling rate for %i bandwidth", p, bandwidth);
        return -1;
    }

    if (rate < bandwidth)
    {
        Log(LogLevel::Error, "Port[%i] Manually specified sample rate %f is less than LTE bandwidth %i", p, rate, bandwidth);
        return -1;
    }
    Log(LogLevel::Info, "Port[%i] Manually specified sample rate: %f MSps", p, rate / 1e6);
    psample_rate->num = rate;
    psample_rate->den = 1;
    *psample_rate_num = rate / 1920000;
    return 0;
}

// return expected number of samples in Tx packet
static int limesuiteng_trx_get_tx_samples_per_packet_func(TRXState* s1)
{
    // This impacts host processing performance at high sampling rates.
    // The limesuiteng API can accept any number of samples, and splits them into
    // multiple packets internally. So this can be any number.
    const int txExpectedSamples = 8192;
    Log(LogLevel::Debug, "Hardware expected samples count in Tx packet : %i", txExpectedSamples);
    return txExpectedSamples;
}

static int limesuiteng_trx_get_abs_rx_power_func(TRXState* s1, float* presult, int channel_num)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    if (lime->rxChannels[channel_num].parent->configInputs.rx.powerAvailable)
    {
        *presult = lime->rxChannels[channel_num].parent->configInputs.rx.power_dBm;
        return 0;
    }
    else
        return -1;
}

static int limesuiteng_trx_get_abs_tx_power_func(TRXState* s1, float* presult, int channel_num)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    if (lime->txChannels[channel_num].parent->configInputs.tx.powerAvailable)
    {
        *presult = lime->txChannels[channel_num].parent->configInputs.tx.power_dBm;
        return 0;
    }
    else
        return -1;
}

//min gain 0
//max gain ~70-76 (higher will probably degrade signal quality to much)
static void limesuiteng_trx_set_tx_gain_func(TRXState* s1, double gain, int channel_num)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    LimePlugin_SetTxGain(lime, gain, channel_num);
}

static void limesuiteng_trx_set_rx_gain_func(TRXState* s1, double gain, int channel_num)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    LimePlugin_SetRxGain(lime, gain, channel_num);
}

static int limesuiteng_trx_set_start_params(TRXState* s1, const TRXDriverParams2* params)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    LimeRuntimeParameters state;

    int rxCount = params->rx_channel_count;
    int txCount = params->tx_channel_count;

    CopyCArrayToVector(state.rx.freq, params->rx_freq, rxCount);
    CopyCArrayToVector(state.tx.freq, params->tx_freq, txCount);

    CopyCArrayToVector(state.rx.gain, params->rx_gain, rxCount);
    CopyCArrayToVector(state.tx.gain, params->tx_gain, txCount);

    CopyCArrayToVector(state.rx.bandwidth, params->rx_bandwidth, rxCount);
    CopyCArrayToVector(state.tx.bandwidth, params->tx_bandwidth, txCount);

    for (int i = 0; i < params->rf_port_count; ++i)
    {
        double sample_rate = static_cast<double>(params->sample_rate[i].num) / params->sample_rate[i].den;
        state.rf_ports.push_back({ sample_rate, params->rx_port_channel_count[i], params->tx_port_channel_count[i] });
    }

    return LimePlugin_Setup(lime, &state);
}

static int limesuiteng_trx_start_func(TRXState* s1, const TRXDriverParams* params)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    LimeRuntimeParameters state;

    int rxCount = params->rx_channel_count;
    int txCount = params->tx_channel_count;

    CopyCArrayToVector(state.rx.freq, params->rx_freq, rxCount);
    CopyCArrayToVector(state.tx.freq, params->tx_freq, txCount);

    CopyCArrayToVector(state.rx.gain, params->rx_gain, rxCount);
    CopyCArrayToVector(state.tx.gain, params->tx_gain, txCount);

    CopyCArrayToVector(state.rx.bandwidth, params->rx_bandwidth, rxCount);
    CopyCArrayToVector(state.tx.bandwidth, params->tx_bandwidth, txCount);

    for (int i = 0; i < params->rf_port_count; ++i)
    {
        double sample_rate = static_cast<double>(params->sample_rate[i].num) / params->sample_rate[i].den;
        state.rf_ports.push_back({ sample_rate, params->rx_port_channel_count[i], params->tx_port_channel_count[i] });
    }

    int status = LimePlugin_Setup(lime, &state);
    if (status != 0)
        return status;
    return LimePlugin_Start(lime);
}

static int limesuiteng_trx_start_func2(TRXState* s1, const TRXDriverParams2* params)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    // params can be NULL if trx_set_start_params() was called before.
    if (params != NULL)
    {
        int ret = limesuiteng_trx_set_start_params(s1, params);
        if (ret != 0)
            return ret;
    }
    return LimePlugin_Start(lime);
}

static void limesuiteng_trx_stop_func(TRXState* s1)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    LimePlugin_Stop(lime);
}

static void limesuiteng_trx_end_func(TRXState* s1)
{
    LimePluginContext* lime = static_cast<LimePluginContext*>(s1->opaque);
    LimePlugin_Destroy(lime);
    delete lime;
}

AmarisoftParamProvider configProvider;

// Driver initialization called at eNB startup
int __attribute__((visibility("default"))) trx_driver_init(TRXState* hostState)
{
    if (hostState->trx_api_version != TRX_API_VERSION)
    {
        fprintf(stderr,
            "ABI compatibility mismatch between LTEENB and TRX driver (LTEENB ABI version=%d, TRX driver ABI version=%d)\n",
            hostState->trx_api_version,
            TRX_API_VERSION);
        return -1;
    }

    LimePluginContext* lime = new LimePluginContext();
    lime->currentWorkingDirectory = std::string(hostState->path);
    lime->samplesFormat = DataFormat::F32;
    configProvider.Init(hostState);

    if (LimePlugin_Init(lime, LogCallback, &configProvider) != 0)
        return -1;

    // Set callbacks
    hostState->opaque = lime;
    hostState->trx_start_func = limesuiteng_trx_start_func; // Obsolete
    hostState->trx_write_func = limesuiteng_trx_write_func; // Deprecated
    hostState->trx_read_func = limesuiteng_trx_read_func; // Deprecated
    hostState->trx_set_tx_gain_func = limesuiteng_trx_set_tx_gain_func;
    hostState->trx_set_rx_gain_func = limesuiteng_trx_set_rx_gain_func;

    hostState->trx_end_func = limesuiteng_trx_end_func;
    hostState->trx_get_tx_samples_per_packet_func = limesuiteng_trx_get_tx_samples_per_packet_func;
    hostState->trx_get_stats = limesuiteng_trx_get_stats;
    hostState->trx_dump_info = limesuiteng_trx_dump_info;
    hostState->trx_get_abs_tx_power_func = limesuiteng_trx_get_abs_tx_power_func;
    hostState->trx_get_abs_rx_power_func = limesuiteng_trx_get_abs_rx_power_func;
    hostState->trx_write_func2 = limesuiteng_trx_write_func2;
    hostState->trx_read_func2 = limesuiteng_trx_read_func2;

    // TODO: get gain
    //hostState->trx_get_tx_gain_func = ;
    //hostState->trx_get_rx_gain_func = ;

    hostState->trx_stop_func = limesuiteng_trx_stop_func;
    hostState->trx_start_func2 = limesuiteng_trx_start_func2;

    // TODO: take higher level loggin either from lime config, or stack's config
    // hostState->trx_log_set_level_func = ;

    hostState->trx_set_start_params = limesuiteng_trx_set_start_params;
    hostState->trx_get_sample_rate_func = limesuiteng_trx_get_sample_rate_func;

    configProvider.Block(); // config parameters access is only allow within trx_driver_init
    return 0;
}
