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
    from gnuradio.drat import udp_source_fc32, udp_source_sc16, udp_source_b
except ImportError:
    import os
    import sys
    dirname, filename = os.path.split(os.path.abspath(__file__))
    sys.path.append(os.path.join(dirname, "bindings"))
    from gnuradio.drat import udp_source


class qa_udp_source(gr_unittest.TestCase):

    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_instance(self):
        samp_rate = 32e3
        instance = udp_source_fc32('0.0.0.0', 1234, samp_rate)
        instance2 = udp_source_sc16('127.0.0.1', 5555, samp_rate,
                                    12345.6, 0x123, True)
        instance3 = udp_source_b('0.0.0.0', 1235, samp_rate)

    def test_source_fc32(self):
        self.source_test('fc32')

    def test_source_sc16(self):
        self.source_test('sc16')

    def test_source_b(self):
        self.source_test('b')

    def source_test(self, output_type):
        ip = '127.0.0.1'
        port = 61234
        samp_rate = 32e3
        source = {'fc32': udp_source_fc32,
                  'sc16': udp_source_sc16,
                  'b': udp_source_b}[output_type](ip, port, samp_rate,
                                                  recv_timeout=100.0)
        sink = (blocks.vector_sink_c() if output_type == 'fc32'
                else blocks.vector_sink_s(2) if output_type == 'sc16'
                else blocks.vector_sink_b())
        self.tb.connect(source, sink)
        self.tb.start()

        num_packets = 64
        payload_lengths = 2 * np.random.randint(50, 2000, num_packets)
        total_len = np.sum(payload_lengths)
        data = [np.random.randint(-2**15, 2**15, length, 'int16')
                if output_type in ['fc32', 'sc16']
                else np.random.randint(0, 256, length, 'uint8')
                for length in payload_lengths]
        seq = 0
        type_ = 0  # INT16_COMPLEX
        flow_id = 1234
        timestamp = 0
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for j, d in enumerate(data):
            if j == len(data) - 1:
                # last packet; set EOB
                type_ = 0x10  # INT16_COMPLEX_EOB
            scalar_size = (2 if output_type in ['fc32', 'sc16']
                           else 1)
            scalars_per_sample = (2 if output_type in ['fc32', 'sc16']
                                  else 1)
            size = 16 + scalar_size * d.size
            header = struct.pack('>BBHIQ', type_, seq, size,
                                 flow_id, timestamp)
            seq = (seq + 1) % 256
            timestamp += d.size // scalars_per_sample
            d_big = d.byteswap() if sys.byteorder == 'little' else d
            packet = header + bytes(d_big)
            time.sleep(0.001)
            sock.sendto(packet, (ip, port))

        time.sleep(0.1)
        sock.close()
        self.tb.stop()
        self.tb.wait()

        all_data = list(np.concatenate(data))
        sink_data = sink.data()
        if output_type == 'fc32':
            sink_data_sc16 = list(np.round(
                2**15 * np.array(sink_data, 'complex').view('float'))
                                  .astype('int16'))
            self.assertEqual(all_data, sink_data_sc16)
        elif output_type in ['sc16', 'b']:
            self.assertEqual(all_data, sink_data)
        else:
            raise ValueError('unknown output_type')

    def test_lost_packets(self):
        self.lost_packets()

    def test_lost_packets_zero_fill(self):
        self.lost_packets(zero_fill_lost=True)

    def lost_packets(self, zero_fill_lost=False):
        ip = '127.0.0.1'
        port = 61234
        samp_rate = 32e3
        source = udp_source_sc16(
            ip, port, samp_rate, zero_fill_lost=zero_fill_lost,
            recv_timeout=10.0)
        sink = blocks.vector_sink_s(2)
        msg = blocks.message_debug()
        self.tb.connect(source, sink)
        self.tb.msg_connect((source, 'lost_packets'), (msg, 'store'))
        self.tb.start()

        num_packets = 32
        packet_len = 200
        packet_payload = b'\xff' * (4 * packet_len)
        to_drop = {3, 7, 9, 10, 11, 15, 16, 21}

        drop_ranges = []
        start_drop = None
        for j in range(num_packets):
            if j in to_drop and start_drop is None:
                start_drop = j
            if start_drop is not None and j not in to_drop:
                drop_ranges.append((start_drop, j))
                start_drop = None
        if start_drop is not None:
            drop_ranges.append((start_drop, num_packets))

        seq = 0
        type_ = 0  # INT16_COMPLEX
        flow_id = 1234
        timestamp = 0
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for j in range(num_packets):
            size = 16 + len(packet_payload)
            header = struct.pack('>BBHIQ', type_, seq, size,
                                 flow_id, timestamp)
            seq = (seq + 1) % 256
            timestamp += packet_len
            packet = header + packet_payload
            if j not in to_drop:
                time.sleep(0.001)
                sock.sendto(packet, (ip, port))

        time.sleep(0.1)
        sock.close()
        self.tb.stop()
        self.tb.wait()

        tags = sink.tags()
        lost_tags = [tag for tag in tags
                     if pmt.symbol_to_string(tag.key) == 'lost_packets']
        self.assertEqual(len(lost_tags), len(drop_ranges))
        for j, tag in enumerate(lost_tags):
            missed_packets = sum([drop_ranges[k][1] - drop_ranges[k][0]
                                  for k in range(j)])
            expected_offset = (
                packet_len * (drop_ranges[j][0] - missed_packets)
                if not zero_fill_lost
                else packet_len * drop_ranges[j][0])
            self.assertEqual(tag.offset, expected_offset)
            self.assertEqual(
                pmt.to_uint64(
                    pmt.dict_ref(tag.value, pmt.intern('total_packets'),
                                 pmt.PMT_NIL)),
                drop_ranges[j][0] - missed_packets + 1)
            self.assertEqual(
                pmt.to_long(
                    pmt.dict_ref(tag.value, pmt.intern('new_seq'),
                                 pmt.PMT_NIL)),
                drop_ranges[j][1])
            self.assertEqual(
                pmt.to_long(
                    pmt.dict_ref(tag.value, pmt.intern('expected_seq'),
                                 pmt.PMT_NIL)),
                drop_ranges[j][0])
            self.assertEqual(
                pmt.to_long(
                    pmt.dict_ref(tag.value, pmt.intern('num_lost'),
                                 pmt.PMT_NIL)),
                drop_ranges[j][1] - drop_ranges[j][0])

        timestamp_tags = [tag for tag in tags
                          if pmt.symbol_to_string(tag.key) == 'rx_time']
        drat_time_tags = [tag for tag in tags
                          if pmt.symbol_to_string(tag.key) == 'drat_time']
        self.assertEqual(len(timestamp_tags), len(drop_ranges) + 1)
        self.assertEqual(len(drat_time_tags), len(drop_ranges) + 1)
        for j, tags in enumerate(zip(timestamp_tags, drat_time_tags)):
            tag, drat_tag = tags
            if j == 0:
                # initial timestamp
                expected_samples = 0
                expected_offset = 0
            else:
                missed_packets = sum([drop_ranges[k][1] - drop_ranges[k][0]
                                      for k in range(j-1)])
                expected_samples = packet_len * drop_ranges[j-1][1]
                expected_offset = (
                    packet_len * (drop_ranges[j-1][0] - missed_packets)
                    if not zero_fill_lost
                    else expected_samples)
            timestamp = (pmt.to_uint64(pmt.tuple_ref(tag.value, 0))
                         + pmt.to_double(pmt.tuple_ref(tag.value, 1)))
            timestamp_samples = round(timestamp * samp_rate)
            self.assertEqual(timestamp_samples, expected_samples)
            self.assertEqual(tag.offset, expected_offset)
            drat_timestamp = pmt.to_uint64(drat_tag.value)
            self.assertEqual(drat_timestamp, expected_samples)
            self.assertEqual(drat_tag.offset, expected_offset)

        messages = [msg.get_message(j)
                    for j in range(msg.num_messages())]
        self.assertEqual(len(messages), len(lost_tags))
        lost_tag_values = [tag.value for tag in
                           sorted(lost_tags, key=lambda t: t.offset)]
        self.assertEqual(lost_tag_values, messages)

        sink_data = sink.data()
        expected_data = []
        for j in range(num_packets):
            if j in to_drop:
                if zero_fill_lost:
                    expected_data.extend([0] * (2 * packet_len))
            else:
                expected_data.extend([-1] * (2 * packet_len))
        self.assertEqual(sink_data, expected_data)

    def test_burst(self):
        ip = '127.0.0.1'
        port = 61234
        samp_rate = 32e3
        source = udp_source_sc16(ip, port, samp_rate, burst_mode=True,
                                 recv_timeout=10.0)
        sink = blocks.vector_sink_s(2)
        self.tb.connect(source, sink)
        self.tb.start()

        num_bursts = 7
        burst_packets = 13
        packet_len = 150
        packet_payload = b'\x00' * (4 * packet_len)

        flow_id = 1234
        timestamp = 0
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for _ in range(num_bursts):
            for j in range(burst_packets):
                size = 16 + len(packet_payload)
                # INT16_COMPLEX or INT16_COMPLEX_EOB
                type_ = 0 if j != burst_packets - 1 else 0x10
                seq = 0 if j == 0 else (seq + 1) % 256
                header = struct.pack('>BBHIQ', type_, seq, size,
                                     flow_id, timestamp)
                timestamp += packet_len
                packet = header + packet_payload
                time.sleep(0.001)
                sock.sendto(packet, (ip, port))

        time.sleep(0.1)
        sock.close()
        self.tb.stop()
        self.tb.wait()

        tags = sink.tags()
        sobs = [tag.offset for tag in tags
                if pmt.symbol_to_string(tag.key) == 'SOB']
        eobs = [tag.offset for tag in tags
                if pmt.symbol_to_string(tag.key) == 'EOB']
        self.assertEqual(sorted(sobs),
                         list(np.arange(num_bursts)
                              * burst_packets * packet_len))
        self.assertEqual(sorted(eobs),
                         list((np.arange(num_bursts) + 1)
                              * burst_packets * packet_len - 1))

    def test_flow_id(self):
        ip = '127.0.0.1'
        port = 61234
        samp_rate = 32e3
        source = udp_source_sc16(ip, port, samp_rate, flow_id=1234,
                                 recv_timeout=10.0)
        sink = blocks.vector_sink_s(2)
        self.tb.connect(source, sink)
        self.tb.start()

        num_packets = 20
        packet_len = 500
        packet_payload = b'\x00' * (4 * packet_len)
        packet_payload2 = b'\xff' * (4 * packet_len)

        type_ = 0  # INT16_COMPLEX
        seq = 0
        timestamp = 0
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for j in range(num_packets):
            size = 16 + len(packet_payload)
            for (flow_id, payload) in zip([1234, 1235],
                                          [packet_payload, packet_payload2]):
                header = struct.pack('>BBHIQ', type_, seq, size,
                                     flow_id, timestamp)
                packet = header + payload
                time.sleep(0.001)
                sock.sendto(packet, (ip, port))
            seq = (seq + 1) % 256
            timestamp += packet_len

        time.sleep(0.1)
        sock.close()
        self.tb.stop()
        self.tb.wait()

        self.assertEqual(sink.data(),
                         [0] * (num_packets * 2 * packet_len))


if __name__ == '__main__':
    gr_unittest.run(qa_udp_source)
