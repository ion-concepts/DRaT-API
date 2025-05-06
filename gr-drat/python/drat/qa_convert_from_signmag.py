#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 gr-drat author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import socket
import struct
import time
import sys

import numpy as np

from gnuradio import gr, gr_unittest, blocks
import pmt
try:
    from gnuradio.drat import (
        convert_from_signmag_sfF, convert_from_signmag_ssF)
except ImportError:
    import os
    import sys
    dirname, filename = os.path.split(os.path.abspath(__file__))
    sys.path.append(os.path.join(dirname, "bindings"))
    from gnuradio.drat import (
        convert_from_signmag_sfF, convert_from_signmag_ssF)


class qa_convert_from_signmag(gr_unittest.TestCase):

    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_instance(self):
        convert_from_signmag_sfF()
        convert_from_signmag_ssF()
        convert_from_signmag_sfF(scale=4.0)
        convert_from_signmag_ssF(scale=0.01)

    def test_conversion_f(self):
        self.conversion_test(dtype='float')

    def test_conversion_s(self):
        self.conversion_test(dtype='int16')

    def conversion_test(self, scale=4.0, dtype='float'):
        dtype_scale = {'float': 1.0, 'int16': 2**15}[dtype]
        convert_block = {'float': convert_from_signmag_sfF,
                         'int16': convert_from_signmag_ssF}[dtype]
        nitems = 4096
        convert = convert_block()
        signs = np.random.randint(0, 256, nitems // 8, 'uint8')
        mags = np.random.randint(0, 256, nitems // 8, 'uint8')
        data = np.zeros(nitems // 4, 'uint8')
        data[::2] = signs
        data[1::2] = mags
        data = data.view('int16')
        source = blocks.vector_source_s(data)
        sink_block = {'float': blocks.vector_sink_f,
                      'int16': blocks.vector_sink_s}[dtype]
        sink = sink_block()
        self.tb.connect(source, convert, sink)
        self.tb.run()

        expected = np.array([1, 3, -3, -1])[
            (np.unpackbits(signs) << 1) | (np.unpackbits(mags))]
        output = np.array(sink.data())
        np.testing.assert_equal(output, expected, 'output does not match')


if __name__ == '__main__':
    gr_unittest.run(qa_convert_from_signmag)
