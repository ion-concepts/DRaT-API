#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 gr-drat author.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import socket
import struct
import threading
import time

import numpy
from gnuradio import gr


class time_source(gr.basic_block):
    """
    DRaT time report packet sender
    """
    def __init__(self, ip, port, samp_rate, flow_id=0,
                 initial_timestamp=0, period=0.1):
        gr.basic_block.__init__(self,
                                name="time_source",
                                in_sig=[],
                                out_sig=[])
        self.ip = ip
        self.port = port
        self.samp_rate = samp_rate
        self.flow_id = flow_id
        self.initial_timestamp = initial_timestamp
        self.period = period
        self.thread = threading.Thread(target=self.send_packets, daemon=True)
        self.thread.start()

    def send_packets(self):
        seq = 0
        type_ = 0xC1  # time report
        size = 16
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        timestamp = self.initial_timestamp
        while True:
            packet = struct.pack('>BBHIQ', type_, seq, size,
                                 self.flow_id, timestamp)
            sock.sendto(packet, (self.ip, self.port))
            time.sleep(self.period)
            seq = (seq + 1) % 256
            timestamp += round(self.period * self.samp_rate)
