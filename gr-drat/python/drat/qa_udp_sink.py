#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 gr-drat author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import socket
import struct
import sys
import threading
import time

import numpy as np

from gnuradio import gr, gr_unittest, blocks
import pmt
try:
    from gnuradio.drat import udp_sink_fc32, udp_sink_sc16, udp_sink_b
except ImportError:
    import os
    import sys
    dirname, filename = os.path.split(os.path.abspath(__file__))
    sys.path.append(os.path.join(dirname, "bindings"))
    from gnuradio.drat import udp_sink_fc32, udp_sink_sc16


class qa_udp_sink(gr_unittest.TestCase):

    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_instance(self):
        samp_rate = 32e3
        flow_id = 0
        packet_size = 1472
        instance = udp_sink_fc32('127.0.0.1', 61234, samp_rate, flow_id,
                                 packet_size)
        instance = udp_sink_sc16('127.0.0.1', 61235, samp_rate, flow_id,
                                 packet_size)
        instance = udp_sink_b('127.0.0.1', 61236, samp_rate, flow_id,
                              packet_size)

    def test_sink_fc32(self):
        self.sink_test('fc32')

    def test_sink_sc16(self):
        self.sink_test('sc16')

    def test_sink_b(self):
        self.sink_test('b')

    def test_sink_fc32_async(self):
        self.sink_test('fc32', async_mode=True)

    def test_sink_sc16_async(self):
        self.sink_test('sc16', async_mode=True)

    def test_sink_b_async(self):
        self.sink_test('b', async_mode=True)

    def test_sink_fc32_time_report(self):
        self.time_report_test('fc32')

    def test_sink_sc16_time_report(self):
        self.time_report_test('sc16')

    def sink_test(self, output_type, async_mode=False):
        ip = '127.0.0.1'
        port = 61234
        flow_id = 1234567
        samp_rate = 32e3
        packet_size = 1472
        # Burst mode is used to guarantee that all the data is sent,
        # but we send a single burst.
        sink = {'fc32': udp_sink_fc32,
                'sc16': udp_sink_sc16,
                'b': udp_sink_b}[output_type](ip, port, samp_rate,
                                              flow_id, packet_size,
                                              self_clocked=True,
                                              async_mode=async_mode,
                                              burst_mode=True)
        data_len = 10000
        data = (np.random.randint(-2**15, 2**15, 2 * data_len, 'int16')
                if output_type in ['fc32', 'sc16']
                else np.random.randint(0, 256, data_len, 'uint8'))
        data_source = (data if output_type in ['sc16', 'b'] else
                       (data / 2**15).astype('float32').view('complex64'))
        tags = [
            # The UDP sink does not actually need the SOB
            # tag, but we include it anyhow
            gr.tag_utils.python_to_tag((
                0, pmt.intern('SOB'),
                pmt.PMT_NIL, pmt.PMT_NIL)),
            gr.tag_utils.python_to_tag((
                data_len - 1, pmt.intern('EOB'),
                pmt.PMT_NIL, pmt.PMT_NIL))]
        if not async_mode:
            start_time = 1234.567
            tags.append(gr.tag_utils.python_to_tag((
                0, pmt.intern('tx_time'),
                pmt.make_tuple(pmt.from_long(int(start_time)),
                               pmt.from_double(start_time - int(start_time))),
                pmt.PMT_NIL)))
        source = {'fc32': blocks.vector_source_c,
                  'sc16': blocks.vector_source_s,
                  'b': blocks.vector_source_b}[output_type](
                      data_source, vlen=2 if output_type == 'sc16' else 1,
                      tags=tags)
        self.tb.connect(source, sink)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))

        recv_data = []
        self.tb.start()
        MTU = 1500
        expected_seq = 0
        while len(recv_data) < len(data):
            packet, _ = sock.recvfrom(MTU)
            scalar_size = (2 if output_type in ['fc32', 'sc16']
                           else 1)
            scalars_per_sample = (2 if output_type in ['fc32', 'sc16']
                                  else 1)
            eob = (scalar_size * (len(data) - len(recv_data))
                   <= packet_size - 16)
            if eob:
                self.assertEqual(
                    len(packet),
                    16 + scalar_size * (len(data) - len(recv_data)))
            else:
                self.assertEqual(len(packet), packet_size)
            type_, seq, size, flow, timestamp = struct.unpack(
                '>BBHIQ', packet[:16])
            not_eob_type = 0x20 if async_mode else 0x0
            eob_type = 0x30 if async_mode else 0x10
            self.assertEqual(type_, eob_type if eob else not_eob_type)
            self.assertEqual(seq, expected_seq)
            expected_seq += 1
            self.assertEqual(size, len(packet))
            self.assertEqual(flow, flow_id)
            if async_mode:
                self.assertEqual(timestamp, 0)
            else:
                expected_timestamp = (
                    start_time * samp_rate
                    + len(recv_data) // scalars_per_sample)
                self.assertEqual(timestamp, expected_timestamp)
            packet_data = np.frombuffer(
                packet[16:],
                'int16' if output_type in ['fc32', 'sc16']
                else 'uint8')
            if sys.byteorder == 'little':
                packet_data = packet_data.byteswap()
            recv_data.extend(packet_data)

        self.tb.wait()
        sock.close()
        self.assertEqual(list(data), recv_data)

    def time_report_test(self, output_type):
        ip = '127.0.0.1'
        port = 61234
        time_report_port = port + 1
        flow_id = 1234567
        samp_rate = 32e3
        packet_size = 1472
        timestamp_epoch = 100_000_000.0
        time_report_flow_id = flow_id + 1
        sink = {'fc32': udp_sink_fc32,
                'sc16': udp_sink_sc16}[output_type](
                    ip, port, samp_rate,
                    flow_id, packet_size,
                    timestamp_epoch=timestamp_epoch,
                    time_report_addr=ip,
                    time_report_port=time_report_port,
                    time_report_flow_id=time_report_flow_id,
                    burst_mode=True)
        burst_len = 10000
        num_burst = 100
        total_len = burst_len * num_burst
        data = np.random.randint(-2**15, 2**15, 2 * total_len, 'int16')
        data_source = (data if output_type == 'sc16' else
                       (data / 2**15).astype('float32').view('complex64'))
        sob_tags = [
            gr.tag_utils.python_to_tag((
                j * burst_len, pmt.intern('SOB'),
                pmt.PMT_NIL, pmt.PMT_NIL))
            for j in range(num_burst)]
        eob_tags = [
            gr.tag_utils.python_to_tag((
                (j + 1) * burst_len - 1, pmt.intern('EOB'),
                pmt.PMT_NIL, pmt.PMT_NIL))
            for j in range(num_burst)]
        start_time = timestamp_epoch + 123567.89
        burst_time_delta = 3 * burst_len / samp_rate
        tx_times = [start_time + j * burst_time_delta
                    for j in range(num_burst)]
        tx_time_tags = [
            gr.tag_utils.python_to_tag((
                j * burst_len, pmt.intern('tx_time'),
                pmt.make_tuple(pmt.from_long(int(tx_times[j])),
                               pmt.from_double(tx_times[j]
                                               - int(tx_times[j]))),
                pmt.PMT_NIL))
            for j in range(num_burst)]
        all_tags = sob_tags + eob_tags + tx_time_tags
        source = {'fc32': blocks.vector_source_c,
                  'sc16': blocks.vector_source_s}[output_type](
                      data_source, vlen=2 if output_type == 'sc16' else 1,
                      tags=all_tags)
        self.tb.connect(source, sink)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, port))

        self.tb.start()
        MTU = 1500

        def send_time_reports():
            timestamp = round((start_time - timestamp_epoch) * samp_rate)
            time_delta = 10e-3
            timestamp_delta = round(samp_rate * time_delta)
            num_to_send = int(
                np.ceil((tx_times[-1] + burst_len / samp_rate - start_time)
                        / time_delta))
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            seq = 0
            for j in range(num_to_send):
                packet = struct.pack(
                    '>BBHIQ', 0xC1, seq, 16, time_report_flow_id,
                    timestamp)
                sock.sendto(packet, (ip, time_report_port))
                timestamp += timestamp_delta
                seq = (seq + 1) % 256
                time.sleep(0.0001)

        timestamp_thread = threading.Thread(target=send_time_reports)
        timestamp_thread.start()

        for j in range(num_burst):
            expected_seq = 0
            recv_data = []
            while len(recv_data) < 2 * burst_len:
                packet, _ = sock.recvfrom(MTU)
                eob = 2 * (2 * burst_len - len(recv_data)) <= packet_size - 16
                if eob:
                    self.assertEqual(len(packet),
                                     16 + 2 * (2 * burst_len - len(recv_data)))
                else:
                    self.assertEqual(len(packet), packet_size)
                type_, seq, size, flow, timestamp = struct.unpack(
                    '>BBHIQ', packet[:16])
                self.assertEqual(type_, 0x10 if eob else 0x00)
                self.assertEqual(seq, expected_seq)
                expected_seq += 1
                self.assertEqual(size, len(packet))
                self.assertEqual(flow, flow_id)
                expected_timestamp = (
                    round((tx_times[j] - timestamp_epoch) * samp_rate)
                    + len(recv_data) // 2)
                self.assertEqual(timestamp, expected_timestamp)
                packet_data = np.frombuffer(packet[16:], 'int16')
                if sys.byteorder == 'little':
                    packet_data = packet_data.byteswap()
                recv_data.extend(packet_data)
            self.assertEqual(recv_data,
                             list(data[2*burst_len*j:2*burst_len*(j+1)]))

        self.tb.wait()
        sock.close()
        timestamp_thread.join()


if __name__ == '__main__':
    gr_unittest.run(qa_udp_sink)
