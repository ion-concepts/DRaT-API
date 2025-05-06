/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DRAT_UDP_SINK_IMPL_H
#define INCLUDED_DRAT_UDP_SINK_IMPL_H

#include "drat.h"
#include <gnuradio/drat/udp_sink.h>
#include <condition_variable>
#include <volk/volk_alloc.hh>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

namespace gr {
namespace drat {

template <class T>
class udp_sink_impl : public udp_sink<T>
{
private:
    int d_socket;
    volk::vector<uint8_t> d_packet_buffer;
    uint8_t* d_curr_pointer;
    const uint8_t* d_end_pointer;
    uint8_t d_seq;
    const int d_ticks_per_sample;
    const double d_timestamp_samp_rate;
    const uint64_t d_timestamp_samp_rate_int;
    const double d_timestamp_samp_rate_frac;
    const uint64_t d_timestamp_epoch_int;
    const double d_timestamp_epoch_frac;
    bool d_need_timestamp;
    bool d_has_timestamp;
    uint64_t d_timestamp;
    const bool d_self_clocked;
    const bool d_async_mode;
    const uint64_t d_maximum_latency_ticks;
    const bool d_burst_mode;
    const pmt::pmt_t d_len_tag_key;
    long d_len_tag_remaining;
    uint64_t d_last_time_report;
    bool d_has_time_report;
    int d_time_report_socket;
    const int64_t d_time_report_flow_id;
    std::thread d_time_report_thread;
    std::mutex d_time_report_mutex;
    std::condition_variable d_time_report_cv;
    std::atomic_bool d_time_report_finish;
    int d_status_socket;
    const int64_t d_status_flow_id;
    std::thread d_status_thread;
    std::atomic_bool d_status_finish;
    std::optional<uint8_t> d_drat_type;
    std::optional<uint8_t> d_drat_type_eob;

    static void pack_samples(drat_packing_t<T>* output_vector,
                             const T* input_vector,
                             size_t num_samples);

    int get_next_eob_index(int nitems, int last_eob_index = -1);

    // Method run by time report thread
    void time_report_thread();

    // Method run by status/error thread
    void status_thread();

    int bind_socket(const std::string& addr, int port, const std::string& name);
    ssize_t recv_drat_packet(int socket,
                             uint8_t* buffer,
                             size_t buffer_len,
                             int64_t flow_id_filter);

public:
    udp_sink_impl(const std::string& dest_addr,
                  int dest_port,
                  double samp_rate,
                  uint32_t flow_id,
                  int packet_size,
                  int ticks_per_sample,
                  double timestamp_epoch,
                  bool self_clocked,
                  bool async_mode,
                  double initial_time,
                  const std::string& time_report_addr,
                  int time_report_port,
                  int64_t time_report_flow_id,
                  const std::string& status_addr,
                  int status_port,
                  int64_t status_flow_id,
                  int maximum_latency,
                  bool burst_mode,
                  const std::string& len_tag_key,
                  int custom_drat_type,
                  int custom_drat_type_eob);
    ~udp_sink_impl() override;

    bool start() override;
    bool stop() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;
};

} // namespace drat
} // namespace gr

#endif /* INCLUDED_DRAT_UDP_SINK_IMPL_H */
