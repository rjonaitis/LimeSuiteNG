/*
 * Copyright 2024 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(sdrdevice_sink.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(30c334ffd641a741a40d08e9d9ede684)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/limesuiteng/sdrdevice_sink.h>
// pydoc.h is automatically generated in the build directory
#include <sdrdevice_sink_pydoc.h>

void bind_sdrdevice_sink(py::module& m)
{

    using sdrdevice_sink = ::gr::limesuiteng::sdrdevice_sink;


    py::class_<sdrdevice_sink,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<sdrdevice_sink>>(m, "sdrdevice_sink", D(sdrdevice_sink))

        .def(py::init(&sdrdevice_sink::make),
             py::arg("alias"),
             py::arg("deviceHandleHint"),
             py::arg("chipIndex"),
             py::arg("channelIndexes"),
             py::arg("dataFormat"),
             py::arg("sampleRate"),
             py::arg("rf_oversampling"),
             D(sdrdevice_sink, make))


        .def("set_config_file",
             &sdrdevice_sink::set_config_file,
             py::arg("file_path"),
             D(sdrdevice_sink, set_config_file))


        .def("set_lo_frequency",
             &sdrdevice_sink::set_lo_frequency,
             py::arg("frequencyHz"),
             D(sdrdevice_sink, set_lo_frequency))


        .def("set_lpf_bandwidth",
             &sdrdevice_sink::set_lpf_bandwidth,
             py::arg("bandwidthHz"),
             D(sdrdevice_sink, set_lpf_bandwidth))


        .def("set_antenna",
             &sdrdevice_sink::set_antenna,
             py::arg("antenna_name"),
             D(sdrdevice_sink, set_antenna))


        .def("set_gain_generic",
             &sdrdevice_sink::set_gain_generic,
             py::arg("gain_dB"),
             D(sdrdevice_sink, set_gain_generic))

        ;
}
