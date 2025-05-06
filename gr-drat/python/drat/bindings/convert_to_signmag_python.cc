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

#include <gnuradio/drat/convert_to_signmag.h>
// pydoc.h is automatically generated in the build directory
#include <convert_to_signmag_pydoc.h>

template <typename T, const bool MSB_FIRST>
void bind_convert_to_signmag_template(py::module& m, const char* classname)
{
    using convert_to_signmag = gr::drat::convert_to_signmag<T, MSB_FIRST>;

    py::class_<convert_to_signmag,
               gr::sync_decimator,
               std::shared_ptr<convert_to_signmag>>(m, classname, D(convert_to_signmag))
        .def(py::init(&convert_to_signmag::make),
             py::arg("vlen_in") = 1,
             py::arg("vlen_out") = 1,
             py::arg("scale") = 1.0,
             D(convert_to_signmag, make))
        .def("scale", &convert_to_signmag::scale, D(convert_to_signmag, scale))
        .def("set_scale",
             &convert_to_signmag::set_scale,
             py::arg("scale"),
             D(convert_to_signmag, scale));
}

void bind_convert_to_signmag(py::module& m)
{
    bind_convert_to_signmag_template<float, false>(m, "convert_to_signmag_fsF");
    bind_convert_to_signmag_template<int32_t, false>(m, "convert_to_signmag_isF");
    bind_convert_to_signmag_template<int16_t, false>(m, "convert_to_signmag_ssF");
    bind_convert_to_signmag_template<int8_t, false>(m, "convert_to_signmag_bsF");
    bind_convert_to_signmag_template<float, true>(m, "convert_to_signmag_fsT");
    bind_convert_to_signmag_template<int32_t, true>(m, "convert_to_signmag_isT");
    bind_convert_to_signmag_template<int16_t, true>(m, "convert_to_signmag_ssT");
    bind_convert_to_signmag_template<int8_t, true>(m, "convert_to_signmag_bsT");
}
