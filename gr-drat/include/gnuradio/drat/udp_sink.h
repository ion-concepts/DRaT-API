/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DRAT_UDP_SINK_H
#define INCLUDED_DRAT_UDP_SINK_H

#include <gnuradio/drat/api.h>
#include <gnuradio/sync_block.h>
#include <cstdint>

namespace gr {
namespace drat {

DRAT_API const pmt::pmt_t tx_time_key();

/*!
 * \brief DRaT UDP Sink
 * \ingroup drat
 *
 */
template <class T>
class DRAT_API udp_sink : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<udp_sink<T>> sptr;

    /*!
     * \brief Create a DRaT UDP Sink block.
     *
     * \param dest_addr Destination IPv4 address.
     * \param dest_port Destination UDP port.
     * \param samp_rate Sample rate.
     * \param flow_id Flow ID.
     * \param packet_size DRaT packet size.
     * \param ticks_per_sample Timestamp ticks per sample.
     * \param timestamp_epoch UNIX timestamp corresponding to the zero timestamp.
     * \param self_clocked Self clocked mode
     *        (does not use backpressure from time reports).
     * \param async_mode Async mode (does not include timestamps in DRaT packets).
     * \param initial_time Initial timestamp.
     * \param time_report_addr Address to listen on for time report packets.
     * \param time_report_port UDP port to listen on for time report packets.
     * \param time_report_flow_id Flow ID filter for time report packets
     *        (if negative, no filtering is done).
     * \param status_addr Address to listen on for status/error packets
     *        (if empty, do not listen).
     * \param status_port UDP port to listen on for status/error packets.
     * \param status_flow_id Flow ID filter for status/error packets
     *        (if negative, no filtering is done).
     * \param maximum_latency Maximum latency (sets how much data can be
     *        sent in advance of a time report).
     * \param burst_mode Burst mode (expects SOB/EOB tags in input).
     * \param len_tag_key Tagged stream packet length tag key.
     * \param custom_drat_type Custom DRaT type for non-EOB packets
              (-1 means use default DRaT type).
     * \param custom_drat_type_eob Custom DRaT type for EOB packets
              (-1 means use default DRaT type).
     */
    static sptr make(const std::string& dest_addr,
                     int dest_port,
                     double samp_rate,
                     uint32_t flow_id,
                     int packet_size,
                     int ticks_per_sample = 1,
                     double timestamp_epoch = 0.0,
                     bool self_clocked = false,
                     bool async_mode = false,
                     double initial_time = 0.0,
                     const std::string& time_report_addr = "",
                     int time_report_port = 0,
                     int64_t time_report_flow_id = -1,
                     const std::string& status_addr = "",
                     int status_port = 0,
                     int64_t status_flow_id = -1,
                     int maximum_latency = 100,
                     bool burst_mode = false,
                     const std::string& len_tag_key = "",
                     int custom_drat_type = -1,
                     int custom_drat_type_eob = -1);
};

typedef udp_sink<gr_complex> udp_sink_fc32;
typedef udp_sink<std::complex<int16_t>> udp_sink_sc16;
typedef udp_sink<float> udp_sink_f;
typedef udp_sink<uint8_t> udp_sink_b;

} // namespace drat
} // namespace gr

#endif /* INCLUDED_DRAT_UDP_SINK_H */
