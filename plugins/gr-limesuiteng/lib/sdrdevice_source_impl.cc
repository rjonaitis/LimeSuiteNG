/* -*- c++ -*- */
/*
 * Copyright 2024 Lime Microsystems Lts <info@limemicro.com>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "sdrdevice_source_impl.h"
#include <gnuradio/io_signature.h>

#include "sdrdevice_manager.h"

#include <chrono>
#include <sstream>
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

sdrdevice_source::sptr sdrdevice_source::make(const std::string& alias,
                                              const std::string& deviceHandleHint,
                                              uint32_t chipIndex,
                                              const std::vector<int>& channelIndexes,
                                              const std::string& dataFormat,
                                              double sampleRate,
                                              int rf_oversampling)
{
    return gnuradio::make_block_sptr<sdrdevice_source_impl>(alias,
                                                            deviceHandleHint,
                                                            chipIndex,
                                                            channelIndexes,
                                                            dataFormat,
                                                            sampleRate,
                                                            rf_oversampling);
}

sdrdevice_source_impl::sdrdevice_source_impl(const std::string& alias,
                                             const std::string& deviceHandleHint,
                                             uint32_t chipIndex,
                                             const std::vector<int>& channelIndexes,
                                             const std::string& dataFormat,
                                             double sampleRate,
                                             int rf_oversampling)
    : gr::sync_block(
          (alias.empty() ? fmt::format("sdrdevice_source[{:s}]", deviceHandleHint.c_str())
                         : alias),
          gr::io_signature::make(0, 0, 0),
          gr::io_signature::make(1 /* min outputs */,
                                 channelIndexes.size() /*max outputs */,
                                 GetDataFormatTypeSize(dataFormat))),
      chipIndex(chipIndex)
{
    devManager = sdrdevice_manager::GetSingleton();
    assert(devManager);
    devContext = devManager->GetDeviceContextByHandle(deviceHandleHint);
    if (!devContext)
        throw std::runtime_error("sdrdevice_source_impl::sdrdevice_source_impl(): No "
                                 "valid device with handle");

    // TODO: allow multiple source instances of the same sdr device, if it use different
    // chip indexe
    if (devContext->sourceBlockCounter > 0)
        throw std::runtime_error("sdrdevice_source_impl: More than one Source block "
                                 "assigned to same SDR device");

    ++devContext->sourceBlockCounter;

    devContext->streamCfg.channels.at(direction).clear();
    for (const auto index : channelIndexes)
        devContext->streamCfg.channels.at(direction).push_back(index);

    devContext->streamCfg.format = GetDataFormatEnum(dataFormat);
    devContext->streamCfg.linkFormat = lime::DataFormat::I16;
    devContext->streamCfg.hintSampleRate = sampleRate;

    lime::SDRConfig& config = devContext->deviceConfig;
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        auto& channel = config.channel[ch].rx;
        channel.enabled = true;
        channel.sampleRate = sampleRate;
        channel.oversample = rf_oversampling;
    }
}

sdrdevice_source_impl::~sdrdevice_source_impl() { GR_LOG_DEBUG(d_logger, __func__); }

bool sdrdevice_source_impl::start()
{
    GR_LOG_DEBUG(d_logger, __func__);
    assert(devContext);

    lime::SDRConfig& config = devContext->deviceConfig;

    devContext->sourceConfigReady = true;

    const bool doConfigure =
        (devContext->sourceBlockCounter ? devContext->sourceConfigReady : true) &&
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

bool sdrdevice_source_impl::stop()
{
    assert(devContext);
    GR_LOG_DEBUG(d_logger, __func__);
    devContext->sourceBlockCounter--;

    const bool doStop =
        (devContext->sourceBlockCounter == 0 && devContext->sinkBlockCounter == 0);

    if (doStop && devContext->stream) {
        devContext->stream->Stop();
        devContext->stream->Teardown();
        devContext->stream.reset();
    }

    // GRC does not call destructor, so cleanup resources on stop() to ensure proper
    // resources destruction order
    this->~sdrdevice_source_impl();
    return true;
}

int sdrdevice_source_impl::work(int noutput_items,
                                gr_vector_const_void_star& input_items,
                                gr_vector_void_star& output_items)
{
    assert(devContext);
    assert(devContext->stream);

    lime::complex32f_t* samples[8];
    for (size_t i = 0; i < devContext->streamCfg.channels.at(direction).size(); ++i)
        samples[i] = static_cast<lime::complex32f_t*>(output_items[i]);

    StreamMeta meta;
    int samplesRead = devContext->stream->StreamRx(
        &samples[0], noutput_items, &meta, std::chrono::microseconds(1000000));

    if (samplesRead != noutput_items)
        GR_LOG_WARN(d_logger,
                    fmt::format("StreamRx {:d}/{:d}", samplesRead / noutput_items));

    return samplesRead;
}

void sdrdevice_source_impl::set_config_file(const std::string& file_path)
{
    GR_LOG_INFO(d_logger, fmt::format("{:s} {:s}", __func__, file_path));
    devContext->customBaseConfigFilepath = file_path;
}

double sdrdevice_source_impl::set_lo_frequency(double frequencyHz)
{
    GR_LOG_INFO(d_logger, fmt::format("{:s} {:f}", __func__, frequencyHz));
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream)
            devContext->device->SetFrequency(chipIndex, direction, ch, frequencyHz);
        else
            devContext->deviceConfig.channel[ch].rx.centerFrequency = frequencyHz;
    }
    return frequencyHz;
}

double sdrdevice_source_impl::set_lpf_bandwidth(double bandwidthHz)
{
    GR_LOG_INFO(d_logger, fmt::format("{:s} {:f}", __func__, bandwidthHz));
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream)
            devContext->device->SetLowPassFilter(chipIndex, direction, ch, bandwidthHz);
        else
            devContext->deviceConfig.channel[ch].rx.lpf = bandwidthHz;
    }
    return bandwidthHz;
}

bool sdrdevice_source_impl::set_antenna(const std::string& antenna_name)
{
    GR_LOG_INFO(d_logger, fmt::format("{:s} {:s}", __func__, antenna_name));
    const auto& antennas =
        devContext->device->GetDescriptor().rfSOC.at(chipIndex).pathNames.at(direction);

    const auto& antennaFind = std::find(antennas.begin(), antennas.end(), antenna_name);
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
            devContext->deviceConfig.channel[ch].rx.path = antennaIndex;
    }
    return true;
}

double sdrdevice_source_impl::set_gain_generic(double gain_dB)
{
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
            devContext->deviceConfig.channel[ch].rx.gain[lime::eGainTypes::GENERIC] =
                gain_dB;
    }
    return gain_dB;
}

double sdrdevice_source_impl::set_nco_frequency(double frequency_offset_Hz)
{
    GR_LOG_INFO(d_logger, fmt::format("{:s} {:f}", __func__, frequency_offset_Hz));
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream)
            devContext->device->SetNCOFrequency(
                chipIndex, direction, ch, 0, frequency_offset_Hz);
        else
            devContext->deviceConfig.channel[ch].rx.NCOoffset = frequency_offset_Hz;
    }
    return frequency_offset_Hz;
}

} /* namespace limesuiteng */
} /* namespace gr */
