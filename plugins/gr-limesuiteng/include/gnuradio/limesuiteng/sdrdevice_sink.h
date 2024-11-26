/* -*- c++ -*- */
/*
 * Copyright 2024 Lime Microsystems Lts <info@limemicro.com>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_LIMESUITENG_SDRDEVICE_SINK_H
#define INCLUDED_LIMESUITENG_SDRDEVICE_SINK_H

#include <gnuradio/limesuiteng/api.h>
#include <gnuradio/sync_block.h>

#include <string>

namespace gr {
namespace limesuiteng {

/*!
 * \brief LimeSDR based device Transmitter block
 * \ingroup limesuiteng
 *
 */
class LIMESUITENG_API sdrdevice_sink : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<sdrdevice_sink> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of limesuiteng::sdrdevice_sink.
     *
     * To avoid accidental use of raw pointers, limesuiteng::sdrdevice_sink's
     * constructor is in a private implementation
     * class. limesuiteng::sdrdevice_sink::make is the public interface for
     * creating new instances.
     */
    static sptr make(const std::string& alias,
                     const std::string& deviceHandleHint,
                     uint32_t chipIndex,
                     const std::vector<int>& channelIndexes,
                     const std::string& dataFormat,
                     double sampleRate,
                     int rf_oversampling);

    /**
     * Set custom configuration file to be used as base settings
     *
     * @param   file_path Filesystem path to file
     */
    virtual void set_config_file(const std::string& file_path) = 0;

    /**
     * Set center frequency
     *
     * @param   frequencyHz Frequency to set in Hz
     * @return  actual center frequency in Hz
     */
    virtual double set_lo_frequency(double frequencyHz) = 0;

    /**
     * Set Analog Low Pass Filter bandwidth
     *
     * @param   bandwidthHz Low Pass Filter bandwidth to set in Hz
     */
    virtual double set_lpf_bandwidth(double bandwidthHz) = 0;

    /**
     * Set which antenna port to use
     *
     * @param   antenna_name Name of the antenna to use, the names can be different among
     * devices
     */
    virtual bool set_antenna(const std::string& antenna_name) = 0;

    /**
     * Set receiver gain
     *
     * @param   gain_dB Gain value in decibels, actual value will be clipped to available
     * range
     * @return  value that was set
     */
    virtual double set_gain_generic(double gain_dB) = 0;
};

} // namespace limesuiteng
} // namespace gr

#endif /* INCLUDED_LIMESUITENG_SDRDEVICE_SINK_H */
