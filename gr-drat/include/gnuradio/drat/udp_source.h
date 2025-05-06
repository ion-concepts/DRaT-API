/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DRAT_UDP_SOURCE_H
#define INCLUDED_DRAT_UDP_SOURCE_H

#include <gnuradio/drat/api.h>
#include <gnuradio/sync_block.h>
#include <cstdint>

namespace gr {
namespace drat {

DRAT_API const pmt::pmt_t lost_packets_key();
DRAT_API const pmt::pmt_t num_lost_key();
DRAT_API const pmt::pmt_t new_seq_key();
DRAT_API const pmt::pmt_t expected_seq_key();
DRAT_API const pmt::pmt_t total_packets_key();
DRAT_API const pmt::pmt_t sob_key();
DRAT_API const pmt::pmt_t eob_key();
DRAT_API const pmt::pmt_t rx_time_key();
DRAT_API const pmt::pmt_t drat_time_key();
DRAT_API const pmt::pmt_t lost_packets_port();
DRAT_API const pmt::pmt_t timestamps_port();

/*!
 * \brief DRaT UDP Source
 * \ingroup drat
 *
 */
template <class T>
class DRAT_API udp_source : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<udp_source<T>> sptr;

    /*!
     * \brief Create a DRaT UDP Source block.
     *
     * \param listen_addr IPv4 address to listen on.
     * \param listen_port UDP port to listen on.
     * \param samp_rate Sample rate for the timestamp ticks.
     * \param timestamp_epoch UNIX timestamp corresponding to the zero timestamp.
     * \param flow_id Flow ID filter (if negative, no filtering is done).
     * \param burst_mode Enable production of SOB/EOB tags in the output.
     * \param status_addr IPv4 address to send status/error packets to
     *        (if empty, status/error is not sent).
     * \param status_port UDP port to send status/error packets to.
     * \param status_flow_id Flow ID to use for status/error packets.
     * \param send_ack_every Controls every how many packets an ACK is sent
     *        (if 0 or negative, sending ACKs is disabled).
     * \param zero_fill_lost Produce zero-filled output samples for lost
     *        packets.
     * \param ticks_per_sample Timestamp ticks per sample.
     * \param recv_timeout Receive timeout in ms.
     * \param custom_drat_type Custom DRaT type for non-EOB packets
              (-1 means use default DRaT type).
     * \param custom_drat_type_eob Custom DRaT type for EOB packets
              (-1 means use default DRaT type).
     */
    static sptr make(const std::string& listen_addr,
                     int listen_port,
                     double samp_rate,
                     double timestamp_epoch = 0.0,
                     int64_t flow_id = -1,
                     bool burst_mode = false,
                     const std::string& status_addr = "",
                     int status_port = 0,
                     uint32_t status_flow_id = 0,
                     int send_ack_every = 0,
                     bool zero_fill_lost = false,
                     int ticks_per_sample = 1,
                     double recv_timeout = -1.0,
                     int custom_drat_type = -1,
                     int custom_drat_type_eob = -1);
};

typedef udp_source<gr_complex> udp_source_fc32;
typedef udp_source<std::complex<int16_t>> udp_source_sc16;
typedef udp_source<float> udp_source_f;
typedef udp_source<uint8_t> udp_source_b;

} // namespace drat
} // namespace gr

#endif /* INCLUDED_DRAT_UDP_SOURCE_H */
