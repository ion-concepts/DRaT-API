/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DRAT_UDP_SOURCE_IMPL_H
#define INCLUDED_DRAT_UDP_SOURCE_IMPL_H

#include "drat.h"
#include <gnuradio/drat/udp_source.h>
#include <volk/volk_alloc.hh>
#include <cstdint>
#include <optional>

namespace gr {
namespace drat {

template <class T>
class udp_source_impl : public udp_source<T>
{
private:
    int d_socket;
    volk::vector<uint8_t> d_packet_buffer;
    uint8_t* d_curr_pointer;
    const uint8_t* d_end_pointer;
    const double d_samp_rate;
    const uint64_t d_samp_rate_int;
    const double d_samp_rate_frac;
    const uint64_t d_timestamp_epoch_int;
    const double d_timestamp_epoch_frac;
    const int64_t d_flow_id;
    const bool d_burst_mode;
    bool d_eob;
    uint8_t d_last_seq;
    uint64_t d_expected_timestamp;
    uint64_t d_total_packets;
    int d_status_socket;
    int d_status_seq;
    const uint32_t d_status_flow_id;
    const int d_send_ack_every;
    const bool d_zero_fill_lost;
    size_t d_to_zero_fill;
    const int d_ticks_per_sample;
    const std::optional<uint8_t> d_drat_type;
    const std::optional<uint8_t> d_drat_type_eob;

    // In little-endian architecture this byteswaps the input vector in-place.
    static void
    unpack_samples(T* output_vector, drat_packing_t<T>* input_vector, size_t num_samples);

    void send_status(enum status_type type,
                     uint64_t timestamp,
                     uint8_t seq0,
                     uint8_t seq1 = 0);

public:
    udp_source_impl(const std::string& listen_addr,
                    int listen_port,
                    double samp_rate,
                    double timestamp_epoch,
                    int64_t flow_id,
                    bool burst_mode,
                    const std::string& status_addr,
                    int status_port,
                    uint32_t status_flow_id,
                    int send_ack_every,
                    bool zero_fill_lost,
                    int ticks_per_sample,
                    double recv_timeout,
                    int custom_drat_type,
                    int custom_drat_type_eob);
    ~udp_source_impl() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace drat
} // namespace gr

#endif /* INCLUDED_DRAT_UDP_SOURCE_IMPL_H */
