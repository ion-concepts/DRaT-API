# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT drat_sources)
    MESSAGE(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT drat_sources)

########################################################################
# Check for pygccxml
########################################################################
GR_PYTHON_CHECK_MODULE_RAW(
    "pygccxml"
    "import pygccxml"
    PYGCCXML_FOUND
    )

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND drat_python_files
    convert_from_signmag_python.cc
    convert_to_signmag_python.cc
    udp_sink_python.cc
    udp_source_python.cc
    python_bindings.cc)

GR_PYBIND_MAKE_OOT(drat
   ../../..
   gr::drat
   "${drat_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(TARGET drat_python POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:drat_python>
    ${PROJECT_BINARY_DIR}/test_modules/gnuradio/drat/
)

install(TARGETS drat_python DESTINATION ${GR_PYTHON_DIR}/gnuradio/drat COMPONENT pythonapi)
