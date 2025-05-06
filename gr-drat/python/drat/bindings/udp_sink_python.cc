/*
 * Copyright 2023 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/drat/udp_sink.h>
// pydoc.h is automatically generated in the build directory
#include <udp_sink_pydoc.h>

template <typename T>
void bind_udp_sink_template(py::module& m, const char* classname)
{
    using udp_sink = ::gr::drat::udp_sink<T>;

    py::class_<udp_sink,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<udp_sink>>(m, classname, D(udp_sink))
        .def(py::init(&udp_sink::make),
             py::arg("dest_addr"),
             py::arg("dest_port"),
             py::arg("samp_rate"),
             py::arg("flow_id"),
             py::arg("packet_size"),
             py::arg("ticks_per_sample") = 1,
             py::arg("timestamp_epoch") = 0.0,
             py::arg("self_clocked") = false,
             py::arg("async_mode") = false,
             py::arg("initial_time") = 0.0,
             py::arg("time_report_addr") = "",
             py::arg("time_report_port") = 0,
             py::arg("time_report_flow_id") = -1,
             py::arg("status_addr") = "",
             py::arg("status_port") = 0,
             py::arg("status_flow_id") = -1,
             py::arg("maximum_latency") = 100,
             py::arg("burst_mode") = false,
             py::arg("len_tag_key") = "",
             py::arg("custom_drat_type") = -1,
             py::arg("custom_drat_type_eob") = -1);
}

void bind_udp_sink(py::module& m)
{
    bind_udp_sink_template<gr_complex>(m, "udp_sink_fc32");
    bind_udp_sink_template<std::complex<int16_t>>(m, "udp_sink_sc16");
    bind_udp_sink_template<float>(m, "udp_sink_f");
    bind_udp_sink_template<uint8_t>(m, "udp_sink_b");
}
