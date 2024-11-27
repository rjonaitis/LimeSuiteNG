/* -*- c++ -*- */
/*
 * Copyright 2024 Lime Microsystems Lts <info@limemicro.com>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sdrdevice_sink_impl.h"
#include <gnuradio/io_signature.h>

#include "sdrdevice_manager.h"

#include <chrono>
#include <thread>

#include "limesuiteng/RFSOCDescriptor.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/complex.h"

using namespace lime;

namespace gr {
namespace limesuiteng {

static int GetDataFormatTypeSize(const std::string& formatname)
{
    if (formatname == "complex16_t")
        return sizeof(lime::complex16_t);
    else if (formatname == "complex12_t")
        return sizeof(lime::complex12_t);
    else if (formatname == "complex32f_t")
        return sizeof(lime::complex32f_t);
    else
        return sizeof(lime::complex32f_t);
}

static lime::DataFormat GetDataFormatEnum(const std::string& formatname)
{
    if (formatname == "complex16_t")
        return lime::DataFormat::I16;
    else if (formatname == "complex12_t")
        return lime::DataFormat::I12;
    else if (formatname == "complex32f_t")
        return lime::DataFormat::F32;
    else
        return lime::DataFormat::F32;
}

sdrdevice_sink::sptr sdrdevice_sink::make(const std::string& alias,
                                          const std::string& deviceHandleHint,
                                          uint32_t chipIndex,
                                          const std::vector<int>& channelIndexes,
                                          const std::string& dataFormat,
                                          double sampleRate,
                                          int rf_oversampling)
{
    return gnuradio::make_block_sptr<sdrdevice_sink_impl>(alias,
                                                          deviceHandleHint,
                                                          chipIndex,
                                                          channelIndexes,
                                                          dataFormat,
                                                          sampleRate,
                                                          rf_oversampling);
}

sdrdevice_sink_impl::sdrdevice_sink_impl(const std::string& alias,
                                         const std::string& deviceHandleHint,
                                         uint32_t chipIndex,
                                         const std::vector<int>& channelIndexes,
                                         const std::string& dataFormat,
                                         double sampleRate,
                                         int rf_oversampling)
    : gr::sync_block((alias.empty()
                          ? fmt::format("sdrdevice_sink[{:s}]", deviceHandleHint.c_str())
                          : alias),
                     gr::io_signature::make(
                         1, channelIndexes.size(), GetDataFormatTypeSize(dataFormat)),
                     gr::io_signature::make(0, 0, 0)),
      chipIndex(chipIndex),
      autoAntenna(true)
{
    devManager = sdrdevice_manager::GetSingleton();
    assert(devManager);
    devContext = devManager->GetDeviceContextByHandle(deviceHandleHint);
    if (!devContext)
        throw std::runtime_error(
            "sdrdevice_sink_impl::sdrdevice_sink_impl(): No valid device with handle");

    // TODO: allow multiple source instances of the same sdr device, if it use different
    // chip indexe
    if (devContext->sinkBlockCounter > 0)
        throw std::runtime_error(
            "sdrdevice_sink_impl: More than one Sink block assigned to same SDR device");

    ++devContext->sinkBlockCounter;

    devContext->streamCfg.channels.at(direction).clear();
    for (const auto index : channelIndexes)
        devContext->streamCfg.channels.at(direction).push_back(index);

    devContext->streamCfg.format = GetDataFormatEnum(dataFormat);
    devContext->streamCfg.linkFormat = lime::DataFormat::I16;
    devContext->streamCfg.hintSampleRate = sampleRate;

    lime::SDRConfig& config = devContext->deviceConfig;
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        auto& channel = config.channel[ch].tx;
        channel.enabled = true;
        channel.sampleRate = sampleRate;
        channel.oversample = rf_oversampling;
    }
}

sdrdevice_sink_impl::~sdrdevice_sink_impl()
{
    GR_LOG_DEBUG(d_logger, __func__);
    ReleaseResources();
}

void sdrdevice_sink_impl::ReleaseResources()
{
    GR_LOG_DEBUG(d_logger, __func__);
    devContext.reset();
    devManager.reset();
}

bool sdrdevice_sink_impl::start()
{
    GR_LOG_DEBUG(d_logger, __func__);
    assert(devContext);

    lime::SDRConfig& config = devContext->deviceConfig;

    devContext->sinkConfigReady = true;
    const bool doConfigure =
        (devContext->sinkBlockCounter ? devContext->sinkConfigReady : true) &&
        (devContext->sinkBlockCounter ? devContext->sinkConfigReady : true);

    if (!doConfigure)
        return true;

    GR_LOG_DEBUG(d_logger, "SDRDevice Configure()");
    config.skipDefaults = !devContext->customBaseConfigFilepath.empty();
    if (config.skipDefaults &&
        devContext->device->LoadConfig(chipIndex, devContext->customBaseConfigFilepath) !=
            OpStatus::Success) {
        const std::string msg = fmt::format("Failed to load configuration file {:s}",
                                            devContext->customBaseConfigFilepath);
        GR_LOG_ERROR(d_logger, msg);
        throw std::runtime_error(msg);
    }

    if (devContext->device->Configure(config, chipIndex) != OpStatus::Success) {
        const std::string msg = fmt::format("Failed to configure device");
        GR_LOG_ERROR(d_logger, msg);
        throw std::runtime_error(msg);
        return false;
    }

    GR_LOG_DEBUG(d_logger, "RFStream Create()");
    devContext->stream =
        devContext->device->StreamCreate(devContext->streamCfg, chipIndex);
    if (!devContext->stream)
        return false;

    GR_LOG_DEBUG(d_logger, "RFStream Start()");
    if (devContext->stream->Start() != OpStatus::Success)
        return false;

    return true;
}

bool sdrdevice_sink_impl::stop()
{
    assert(devContext);
    GR_LOG_DEBUG(d_logger, __func__);
    devContext->sinkBlockCounter--;
    const bool doStop =
        (devContext->sourceBlockCounter == 0 && devContext->sinkBlockCounter == 0);

    if (doStop && devContext->stream) {
        GR_LOG_DEBUG(d_logger, "RFStream Stop");
        devContext->stream->Stop();
        devContext->stream->Teardown();
        devContext->stream.reset();
    }

    // GRC does not call destructor, so cleanup resources on stop() to ensure proper
    // resources destruction order
    ReleaseResources();
    return true;
}

int sdrdevice_sink_impl::work(int noutput_items,
                              gr_vector_const_void_star& input_items,
                              gr_vector_void_star& output_items)
{
    assert(devContext);
    assert(devContext->stream);

    const lime::complex32f_t* samples[8];
    for (size_t i = 0; i < devContext->streamCfg.channels.at(direction).size(); ++i)
        samples[i] = static_cast<const lime::complex32f_t*>(input_items[i]);

    StreamMeta meta;
    meta.waitForTimestamp = false;
    meta.flushPartialPacket = true;
    int samplesSent = devContext->stream->StreamTx(
        &samples[0], noutput_items, &meta, std::chrono::microseconds(1000000));

    if (samplesSent != noutput_items)
        GR_LOG_WARN(d_logger,
                    fmt::format("StreamTx {:d}/{:d}", samplesSent / noutput_items));

    return samplesSent;
}

void sdrdevice_sink_impl::set_config_file(const std::string& file_path)
{
    if (!devContext)
        return;

    GR_LOG_INFO(d_logger, fmt::format("{:s} {:s}", __func__, file_path));
    devContext->customBaseConfigFilepath = file_path;
}

double sdrdevice_sink_impl::set_lo_frequency(double frequencyHz)
{
    if (!devContext)
        return frequencyHz;

    GR_LOG_INFO(d_logger, fmt::format("{:s} {:f}", __func__, frequencyHz));
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream)
            devContext->device->SetFrequency(chipIndex, direction, ch, frequencyHz);
        else
            devContext->deviceConfig.channel[ch].tx.centerFrequency = frequencyHz;

        if (autoAntenna)
            set_antenna("auto");
    }
    return frequencyHz;
}

double sdrdevice_sink_impl::set_lpf_bandwidth(double bandwidthHz)
{
    if (!devContext)
        return bandwidthHz;

    GR_LOG_INFO(d_logger, fmt::format("{:s} {:f}", __func__, bandwidthHz));
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream)
            devContext->device->SetLowPassFilter(chipIndex, direction, ch, bandwidthHz);
        else
            devContext->deviceConfig.channel[ch].tx.lpf = bandwidthHz;
    }
    return bandwidthHz;
}

static std::string
GetAntennaForFrequency(double frequencyHz,
                       const std::vector<std::string>& options,
                       const std::unordered_map<std::string, lime::Range<double>>& ranges)
{
    std::string name = "auto_failed";
    float deviation = 1e20;
    for (const auto& name_range : ranges) {
        const auto& range = name_range.second;

        if (std::find(options.begin(), options.end(), name_range.first) == options.end())
            continue;

        const double mid = range.min + (range.max - range.min) / 2;
        const double d = std::abs(mid - frequencyHz);
        if (d <= deviation) {
            name = name_range.first;
            deviation = d;
        }
    }
    return name;
}

bool sdrdevice_sink_impl::set_antenna(const std::string& antenna_name)
{
    if (!devContext)
        return false;

    GR_LOG_INFO(d_logger, fmt::format("{:s} {:s}", __func__, antenna_name));
    const auto& antennas =
        devContext->device->GetDescriptor().rfSOC.at(chipIndex).pathNames.at(direction);
    auto antennaFind = antennas.begin();

    autoAntenna = antenna_name.empty() || (antenna_name == "auto");

    if (autoAntenna) {
        double freq;
        int ch = devContext->streamCfg.channels.at(direction).front();
        if (devContext->stream)
            freq = devContext->device->GetFrequency(chipIndex, direction, ch);
        else
            freq = devContext->deviceConfig.channel[ch].tx.centerFrequency;
        const std::string bestAntenna = GetAntennaForFrequency(
            freq,
            antennas,
            devContext->device->GetDescriptor().rfSOC.at(chipIndex).antennaRange.at(
                direction));
        GR_LOG_INFO(d_logger, fmt::format("auto selected antenna: {:s}", bestAntenna));
        antennaFind = std::find(antennas.begin(), antennas.end(), bestAntenna);
    } else
        antennaFind = std::find(antennas.begin(), antennas.end(), antenna_name);

    if (antennaFind == antennas.end()) {
        std::stringstream ss;
        ss << "Antenna " << antenna_name << " not found. Available:" << std::endl;
        for (const auto& iter : antennas)
            ss << "\t\"" << iter << "\"" << std::endl;
        GR_LOG_ERROR(d_logger, ss.str());
        return false;
    }

    const int antennaIndex = std::distance(antennas.begin(), antennaFind);
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream)
            devContext->device->SetAntenna(chipIndex, direction, ch, antennaIndex);
        else
            devContext->deviceConfig.channel[ch].tx.path = antennaIndex;
    }
    return true;
}

double sdrdevice_sink_impl::set_gain_generic(double gain_dB)
{
    if (!devContext)
        return gain_dB;

    GR_LOG_INFO(d_logger, fmt::format("{:s} {:f}", __func__, gain_dB));
    const RFSOCDescriptor& desc = devContext->device->GetDescriptor().rfSOC.at(chipIndex);

    lime::Range gain_range = desc.gainRange.at(direction).at(lime::eGainTypes::GENERIC);
    GR_LOG_INFO(
        d_logger,
        fmt::format(
            "{:s} gain range: {:f}:{:f}", __func__, gain_range.min, gain_range.max));
    if (gain_dB > gain_range.max) {
        gain_dB = gain_range.max;
        GR_LOG_WARN(d_logger, fmt::format("{:s} clip gain to {:f}", __func__, gain_dB));
    } else if (gain_dB < gain_range.min) {
        gain_dB = gain_range.min;
        GR_LOG_WARN(d_logger, fmt::format("{:s} clip gain to {:f}", __func__, gain_dB));
    }

    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream)
            devContext->device->SetGain(
                chipIndex, direction, ch, lime::eGainTypes::GENERIC, gain_dB);
        else
            devContext->deviceConfig.channel[ch].tx.gain[lime::eGainTypes::GENERIC] =
                gain_dB;
    }
    return gain_dB;
}

double sdrdevice_sink_impl::set_nco_frequency(double frequency_offset_Hz)
{
    if (!devContext)
        return frequency_offset_Hz;

    GR_LOG_INFO(d_logger, fmt::format("{:s} {:f}", __func__, frequency_offset_Hz));
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream)
            devContext->device->SetNCOFrequency(
                chipIndex, direction, ch, 0, frequency_offset_Hz);
        else
            devContext->deviceConfig.channel[ch].tx.NCOoffset = frequency_offset_Hz;
    }
    return frequency_offset_Hz;
}

} /* namespace limesuiteng */
} /* namespace gr */
