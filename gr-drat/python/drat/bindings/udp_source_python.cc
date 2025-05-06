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

#include <gnuradio/drat/udp_source.h>
// pydoc.h is automatically generated in the build directory
#include <udp_source_pydoc.h>

template <typename T>
void bind_udp_source_template(py::module& m, const char* classname)
{
    using udp_source = ::gr::drat::udp_source<T>;

    py::class_<udp_source,
               gr::sync_block,
               gr::block,
               gr::basic_block,
               std::shared_ptr<udp_source>>(m, classname, D(udp_source))
        .def(py::init(&udp_source::make),
             py::arg("listen_addr"),
             py::arg("listen_port"),
             py::arg("samp_rate"),
             py::arg("timestamp_epoch") = 0.0,
             py::arg("flow_id") = -1,
             py::arg("burst_mode") = false,
             py::arg("status_addr") = "",
             py::arg("status_port") = 0,
             py::arg("status_flow_id") = 0,
             py::arg("send_ack_every") = 0,
             py::arg("zero_fill_lost") = false,
             py::arg("ticks_per_sample") = 1,
             py::arg("recv_timeout") = -1.0,
             py::arg("custom_drat_type") = -1,
             py::arg("custom_drat_type_eob") = -1,
             D(udp_source));
}

void bind_udp_source(py::module& m)
{
    bind_udp_source_template<gr_complex>(m, "udp_source_fc32");
    bind_udp_source_template<std::complex<int16_t>>(m, "udp_source_sc16");
    bind_udp_source_template<float>(m, "udp_source_f");
    bind_udp_source_template<uint8_t>(m, "udp_source_b");

    m.def("lost_packets_key", &::gr::drat::lost_packets_key, D(lost_packets_key));
    m.def("num_lost_key", &::gr::drat::num_lost_key, D(num_lost_key));
    m.def("new_seq_key", &::gr::drat::new_seq_key, D(new_seq_key));
    m.def("expected_seq_key", &::gr::drat::expected_seq_key, D(expected_seq_key));
    m.def("total_packets_key", &::gr::drat::total_packets_key, D(total_packets_key));
    m.def("sob_key", &::gr::drat::sob_key, D(sob_key));
    m.def("eob_key", &::gr::drat::eob_key, D(eob_key));
    m.def("rx_time_key", &::gr::drat::rx_time_key, D(rx_time_key));
    m.def("drat_time_key", &::gr::drat::drat_time_key, D(drat_time_key));
    m.def("timestamps_port", &::gr::drat::timestamps_port, D(timestamps_port));
    m.def("lost_packets_port", &::gr::drat::lost_packets_port, D(lost_packets_port));
}
