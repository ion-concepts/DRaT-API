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

#include <gnuradio/drat/convert_from_signmag.h>
// pydoc.h is automatically generated in the build directory
#include <convert_from_signmag_pydoc.h>

template <typename T, const bool MSB_FIRST>
void bind_convert_from_signmag_template(py::module& m, const char* classname)
{
    using convert_from_signmag = gr::drat::convert_from_signmag<T, MSB_FIRST>;

    py::class_<convert_from_signmag,
               gr::sync_interpolator,
               std::shared_ptr<convert_from_signmag>>(
        m, classname, D(convert_from_signmag))
        .def(py::init(&convert_from_signmag::make),
             py::arg("vlen_in") = 1,
             py::arg("vlen_out") = 1,
             py::arg("scale") = 1.0,
             D(convert_from_signmag, make))
        .def("scale", &convert_from_signmag::scale, D(convert_from_signmag, scale))
        .def("set_scale",
             &convert_from_signmag::set_scale,
             py::arg("scale"),
             D(convert_from_signmag, scale));
}

void bind_convert_from_signmag(py::module& m)
{
    bind_convert_from_signmag_template<float, false>(m, "convert_from_signmag_sfF");
    bind_convert_from_signmag_template<int32_t, false>(m, "convert_from_signmag_siF");
    bind_convert_from_signmag_template<int16_t, false>(m, "convert_from_signmag_ssF");
    bind_convert_from_signmag_template<int8_t, false>(m, "convert_from_signmag_sbF");
    bind_convert_from_signmag_template<float, true>(m, "convert_from_signmag_sfT");
    bind_convert_from_signmag_template<int32_t, true>(m, "convert_from_signmag_siT");
    bind_convert_from_signmag_template<int16_t, true>(m, "convert_from_signmag_ssT");
    bind_convert_from_signmag_template<int8_t, true>(m, "convert_from_signmag_sbT");
}
