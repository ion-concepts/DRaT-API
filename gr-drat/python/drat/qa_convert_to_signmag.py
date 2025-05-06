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
    from gnuradio.drat import convert_to_signmag_fsF, convert_to_signmag_ssF
except ImportError:
    import os
    import sys
    dirname, filename = os.path.split(os.path.abspath(__file__))
    sys.path.append(os.path.join(dirname, "bindings"))
    from gnuradio.drat import convert_to_signmag_fsF, convert_to_signmag_ssF


class qa_convert_to_signmag(gr_unittest.TestCase):

    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_instance(self):
        convert_to_signmag_fsF()
        convert_to_signmag_ssF()
        convert_to_signmag_fsF(scale=4.0)
        convert_to_signmag_ssF(scale=0.01)

    def test_conversion_f(self):
        self.conversion_test(dtype='float')

    def test_conversion_s(self):
        self.conversion_test(dtype='int16')

    def conversion_test(self, scale=4.0, dtype='float'):
        dtype_scale = {'float': 1.0, 'int16': 2**15}[dtype]
        conversion_scale = scale / dtype_scale
        convert_block = {'float': convert_to_signmag_fsF,
                         'int16': convert_to_signmag_ssF}[dtype]
        nitems = 4096
        convert = convert_block(scale=conversion_scale)
        data = np.random.uniform(-1.0, 1.0, nitems) * dtype_scale
        if dtype in ['int16']:
            data = np.round(data).astype(dtype)
        source_block = {'float': blocks.vector_source_f,
                        'int16': blocks.vector_source_s}[dtype]
        source = source_block(data)
        sink = blocks.vector_sink_s()
        self.tb.connect(source, convert, sink)
        self.tb.run()

        signs = np.packbits(data < 0)
        magnitude_threshold = 2.0 / conversion_scale
        magnitudes = (
            np.packbits(np.abs(data) >= magnitude_threshold)
            ^ signs)
        output = np.array(sink.data(), 'int16').view('uint8')
        np.testing.assert_equal(output[::2], signs, 'signs do not match')
        np.testing.assert_equal(output[1::2], magnitudes,
                                'magnitudes do not match')


if __name__ == '__main__':
    gr_unittest.run(qa_convert_to_signmag)
