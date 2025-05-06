/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "drat.h"
#include "udp_sink_impl.h"
#include <gnuradio/drat/udp_source.h>
#include <gnuradio/io_signature.h>
#include <arpa/inet.h>
#include <cmake_endian.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

using namespace std::chrono_literals;

namespace gr {
namespace drat {

const pmt::pmt_t tx_time_key()
{
    static const auto key = pmt::string_to_symbol("tx_time");
    return key;
}

template <class T>
typename udp_sink<T>::sptr udp_sink<T>::make(const std::string& dest_addr,
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
                                             int custom_drat_type_eob)
{
    return gnuradio::make_block_sptr<udp_sink_impl<T>>(dest_addr,
                                                       dest_port,
                                                       samp_rate,
                                                       flow_id,
                                                       packet_size,
                                                       ticks_per_sample,
                                                       timestamp_epoch,
                                                       self_clocked,
                                                       async_mode,
                                                       initial_time,
                                                       time_report_addr,
                                                       time_report_port,
                                                       time_report_flow_id,
                                                       status_addr,
                                                       status_port,
                                                       status_flow_id,
                                                       maximum_latency,
                                                       burst_mode,
                                                       len_tag_key,
                                                       custom_drat_type,
                                                       custom_drat_type_eob);
}


template <class T>
udp_sink_impl<T>::udp_sink_impl(const std::string& dest_addr,
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
                                int custom_drat_type_eob)
    : gr::sync_block("udp_sink",
                     gr::io_signature::make(1, 1, sizeof(T)),
                     gr::io_signature::make(0, 0, 0)),
      d_packet_buffer(packet_size, 0),
      d_curr_pointer(d_packet_buffer.data()),
      d_end_pointer(&*d_packet_buffer.end()),
      d_seq(0),
      d_ticks_per_sample(ticks_per_sample),
      d_timestamp_samp_rate(samp_rate * ticks_per_sample),
      d_timestamp_samp_rate_int(static_cast<uint64_t>(d_timestamp_samp_rate)),
      d_timestamp_samp_rate_frac(d_timestamp_samp_rate -
                                 static_cast<double>(d_timestamp_samp_rate_int)),
      d_timestamp_epoch_int(static_cast<uint64_t>(timestamp_epoch)),
      d_timestamp_epoch_frac(timestamp_epoch -
                             static_cast<double>(d_timestamp_epoch_int)),
      // In self_clocked mode with burst_mode and tagged stream mode disabled we
      // have the timestamp given by initial_time.
      d_need_timestamp(!self_clocked ||
                       (!async_mode && (burst_mode || !len_tag_key.empty()))),
      d_has_timestamp(self_clocked && !burst_mode && len_tag_key.empty() && !async_mode),
      // Initial time indicates UNIX time. The initial time is only used in
      // self-clocked not async mode not burst mode / tagged stream mode.
      d_timestamp(self_clocked && !burst_mode && len_tag_key.empty() && !async_mode
                      ? round((initial_time - timestamp_epoch) * d_timestamp_samp_rate)
                      : 0),
      d_self_clocked(self_clocked),
      // Async mode is only allowed in self-clocked mode
      d_async_mode(async_mode && self_clocked),
      d_maximum_latency_ticks(
          round(maximum_latency * 1e-3 * samp_rate * ticks_per_sample)),
      d_burst_mode(burst_mode),
      // Burst mode takes precedence over len_tag_key
      d_len_tag_key(len_tag_key.empty() || burst_mode
                        ? pmt::PMT_NIL
                        : pmt::string_to_symbol(len_tag_key)),
      d_len_tag_remaining(0),
      d_last_time_report(0),
      d_has_time_report(false),
      d_time_report_flow_id(time_report_flow_id),
      d_status_flow_id(status_flow_id),
      d_drat_type(custom_drat_type >= 0 ? std::optional<uint8_t>{ custom_drat_type }
                                        : std::nullopt),
      d_drat_type_eob(custom_drat_type_eob >= 0
                          ? std::optional<uint8_t>{ custom_drat_type_eob }
                          : std::nullopt)
{
    if (static_cast<size_t>(packet_size) < DRAT_HEADER_LEN + sizeof(drat_packing_t<T>)) {
        throw std::runtime_error("packet size too small");
    }

    d_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (d_socket < 0) {
        throw std::runtime_error("could not create UDP socket");
    }

    struct sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr(dest_addr.c_str());
    address.sin_port = htons(dest_port);
    if (connect(d_socket,
                reinterpret_cast<const struct sockaddr*>(&address),
                sizeof(address)) < 0) {
        std::perror("could not connect UDP socket");
        throw std::runtime_error("unable to connect UDP socket");
    }

    // The flow ID for output packets is set here and never overwritten.
    *reinterpret_cast<uint32_t*>(&d_packet_buffer[DRAT_FLOW_ID_OFFSET]) = htonl(flow_id);

    if (!self_clocked) {
        d_time_report_socket =
            bind_socket(time_report_addr, time_report_port, "time report");
    }

    if (!status_addr.empty()) {
        d_status_socket = bind_socket(status_addr, status_port, "status/error");
    } else {
        d_status_socket = -1;
    }
}

template <class T>
bool udp_sink_impl<T>::start()
{
    if (!d_self_clocked) {
        d_time_report_finish = false;
        d_time_report_thread = std::thread([this]() { time_report_thread(); });
    }

    if (d_status_socket >= 0) {
        d_status_finish = false;
        d_status_thread = std::thread([this]() { status_thread(); });
    }

    return true;
}

template <class T>
bool udp_sink_impl<T>::stop()
{
    if (!d_self_clocked) {
        d_time_report_finish.store(true, std::memory_order_relaxed);
        d_time_report_thread.join();
    }

    if (d_status_socket >= 0) {
        d_status_finish.store(true, std::memory_order_relaxed);
        d_status_thread.join();
    }

    return true;
}

template <class T>
udp_sink_impl<T>::~udp_sink_impl()
{
    close(d_socket);
    if (!d_self_clocked) {
        close(d_time_report_socket);
    }
    if (d_status_socket >= 0) {
        close(d_status_socket);
    }
}

template <class T>
int udp_sink_impl<T>::work(int noutput_items,
                           gr_vector_const_void_star& input_items,
                           gr_vector_void_star& output_items)
{
    auto in = static_cast<const T*>(input_items[0]);
    const auto in_end = in + noutput_items;
    int consumed = 0;

    // eob_index for tagged stream is handled below
    int eob_index = d_burst_mode ? get_next_eob_index(noutput_items) : noutput_items;

    while (in < in_end) {
        if (d_need_timestamp) {
            d_need_timestamp = false;
            // First try to obtain a drat_time timestamp
            std::vector<gr::tag_t> tags;
            const auto nitems = this->nitems_read(0) + consumed;
            this->get_tags_in_range(tags, 0, nitems, nitems + 1, drat_time_key());
            if (!tags.empty()) {
                d_timestamp = pmt::to_uint64(tags[0].value);
                d_has_timestamp = true;
            } else {
                this->get_tags_in_range(tags, 0, nitems, nitems + 1, tx_time_key());
                if (!tags.empty()) {
                    // If a drat_time tag is not found, fallback to trying to
                    // find a tx_time tag
                    uint64_t t_int = pmt::to_uint64(pmt::tuple_ref(tags[0].value, 0));
                    double t_frac = pmt::to_double(pmt::tuple_ref(tags[0].value, 1));
                    t_int -= d_timestamp_epoch_int;
                    t_frac -= d_timestamp_epoch_frac;
                    const uint64_t bulk_ticks = t_int * d_timestamp_samp_rate_int;
                    const uint64_t more_ticks = static_cast<uint64_t>(
                        round(static_cast<double>(t_int) * d_timestamp_samp_rate_frac +
                              t_frac * d_timestamp_samp_rate));
                    d_timestamp = bulk_ticks + more_ticks;
                    d_has_timestamp = true;
                } else if (d_self_clocked) {
                    // In self-clocked mode timestamps cannot be fetched from time
                    // reports, so this is a runtime error
                    throw std::runtime_error(
                        "[DRaT UDP sink] tx_time tag for burst missing");
                }
            }
        }

        if (!pmt::is_null(d_len_tag_key)) {
            if (d_len_tag_remaining == 0) {
                // fetch a new len_tag key
                std::vector<gr::tag_t> tags;
                this->get_tags_in_range(tags,
                                        0,
                                        this->nitems_read(0) + consumed,
                                        this->nitems_read(0) + consumed + 1,
                                        d_len_tag_key);
                if (tags.empty()) {
                    throw std::runtime_error("[DRaT UDP sink] len_tag key missing");
                }
                d_len_tag_remaining = pmt::to_long(tags[0].value);
                if (d_len_tag_remaining <= 0) {
                    throw std::runtime_error("[DRaT UDP sink] got a len_tag <= 0");
                }
            }
            eob_index = consumed + d_len_tag_remaining - 1;
        }

        if (d_curr_pointer == d_packet_buffer.data()) {
            // Need to create a new DRaT header

            // _EOB type will be set when sending the packet if needed.
            uint8_t drat_type;
            if (d_drat_type.has_value()) {
                drat_type = *d_drat_type;
            } else {
                drat_type =
                    static_cast<uint8_t>(d_async_mode ? packet_type::INT16_COMPLEX_ASYNC
                                                      : packet_type::INT16_COMPLEX);
            }
            d_packet_buffer[DRAT_TYPE_OFFSET] = drat_type;

            d_packet_buffer[DRAT_SEQ_OFFSET] = d_seq;
            ++d_seq;

            // The size field is only set immediately before sending the packet
            // The flow ID field has already been set in the constructor

            if (!d_async_mode) {
                // This uses htobe64(), which is not very portable. std::byteswap() is
                // available in C++23.
                *reinterpret_cast<uint64_t*>(&d_packet_buffer[DRAT_TIMESTAMP_OFFSET]) =
                    htobe64(d_timestamp);
            }

            d_curr_pointer = &d_packet_buffer[DRAT_HEADER_LEN];
        }

        const size_t can_produce =
            (d_end_pointer - d_curr_pointer) / sizeof(drat_packing_t<T>);
        const size_t can_consume =
            std::min(in_end - in, static_cast<long>(eob_index - consumed + 1));
        const auto nitems = std::min(can_produce, can_consume);
        pack_samples(reinterpret_cast<drat_packing_t<T>*>(d_curr_pointer), in, nitems);
        in += nitems;
        consumed += nitems;
        d_curr_pointer += nitems * sizeof(drat_packing_t<T>);
        if (!pmt::is_null(d_len_tag_key)) {
            d_len_tag_remaining -= nitems;
        }

        const bool is_eob = consumed == eob_index + 1;
        if ((d_curr_pointer == d_end_pointer) || is_eob) {
            // Packet complete. Prepare to send it.

            if (is_eob) {
                // This is an EOB packet
                uint8_t drat_type;
                if (d_drat_type_eob.has_value()) {
                    drat_type = *d_drat_type_eob;
                } else {
                    drat_type = static_cast<uint8_t>(
                        d_async_mode ? packet_type::INT16_COMPLEX_ASYNC_EOB
                                     : packet_type::INT16_COMPLEX_EOB);
                }
                d_packet_buffer[DRAT_TYPE_OFFSET] = drat_type;
                if (d_burst_mode) {
                    eob_index = get_next_eob_index(noutput_items, eob_index);
                }
                // The first packet in the next burst gets a seq of zero.
                d_seq = 0;
            }

            // We need to set the packet field first
            const uint16_t packet_len = d_curr_pointer - d_packet_buffer.data();
            *reinterpret_cast<uint16_t*>(&d_packet_buffer[DRAT_SIZE_OFFSET]) =
                htons(packet_len);

            if (!d_async_mode) {
                const int nsamples =
                    (packet_len - DRAT_HEADER_LEN) / sizeof(drat_packing_t<T>);
                const int packet_ticks = nsamples * d_ticks_per_sample;

                if (!d_has_timestamp) {
                    // Wait for a time report to get a suitable timestamp
                    {
                        std::unique_lock lock(d_time_report_mutex);
                        if (!d_time_report_cv.wait_for(
                                lock, 1s, [this]() { return d_has_time_report; })) {
                            // We timeout and return from work() to prevent waiting
                            // forever if no time report packets are arriving.
                            break;
                        }
                        // The timestamp needs to grow with respect to the
                        // timestamp one sample past the end of the previous
                        // burst (which is currently in d_timestamp).
                        d_timestamp = std::max(
                            d_timestamp,
                            d_last_time_report + d_maximum_latency_ticks - packet_ticks);
                    }
                    // Fill packet header with new timestamp
                    *reinterpret_cast<uint64_t*>(
                        &d_packet_buffer[DRAT_TIMESTAMP_OFFSET]) = htobe64(d_timestamp);
                    d_has_timestamp = true;
                }

                // Update timestamp (d_timestamp will now be the timestamp for the next
                // packet)
                d_timestamp += packet_ticks;

                // Wait until we can send the packet
                if (!d_self_clocked) {
                    std::unique_lock lock(d_time_report_mutex);
                    if (!d_time_report_cv.wait_for(lock, 1s, [this]() {
                            return d_last_time_report + d_maximum_latency_ticks >=
                                   d_timestamp;
                        })) {
                        // We timeout and return from work() to prevent waiting
                        // forever if no time report packets are arriving.
                        break;
                    }
                }
            }

            if (send(d_socket, d_packet_buffer.data(), packet_len, 0) < 0) {
                // This can happen if the remote end is sending ICMP destination
                // unreachable (port unreachable), so we simply print an error
                // but do not fail.
                std::perror("[DRaT UDP Sink] could not send to UDP socket");
            }

            // Set the current pointer to the beginning of the buffer so that we
            // can create a new header on the next loop iteration (or work()
            // call).
            d_curr_pointer = d_packet_buffer.data();

            if (!d_async_mode && is_eob) {
                // We need to get a timestamp for the beginning of the next
                // burst.
                d_need_timestamp = true;
                d_has_timestamp = false;
            }
        }
    }

    return consumed;
}

template <class T>
void udp_sink_impl<T>::time_report_thread()
{
    uint8_t packet[DRAT_MAX_MTU];

    while (!d_time_report_finish.load(std::memory_order_relaxed)) {
        const auto len = recv_drat_packet(
            d_time_report_socket, packet, sizeof(packet), d_time_report_flow_id);
        if (len < 0) {
            continue;
        }

        if (packet_type(packet[DRAT_TYPE_OFFSET]) != packet_type::TIME_REPORT) {
            continue;
        }

        const uint64_t time_report =
            be64toh(*reinterpret_cast<uint64_t*>(&packet[DRAT_TIMESTAMP_OFFSET]));
        {
            std::lock_guard lock(d_time_report_mutex);
            d_last_time_report = time_report;
            d_has_time_report = true;
        }
        d_time_report_cv.notify_one();
    }
}

template <class T>
void udp_sink_impl<T>::status_thread()
{
    uint8_t packet[DRAT_MAX_MTU];

    while (!d_status_finish.load(std::memory_order_relaxed)) {
        const auto len =
            recv_drat_packet(d_status_socket, packet, sizeof(packet), d_status_flow_id);
        if (len < 0) {
            continue;
        }

        if (packet_type(packet[DRAT_TYPE_OFFSET]) != packet_type::STATUS) {
            continue;
        }

        if (len != 24) {
            this->d_logger->error("received status of unexpected length {}", len);
        }

        const status_type type =
            status_type(ntohl(*reinterpret_cast<uint32_t*>(&packet[DRAT_HEADER_LEN])));
        const uint8_t seq0 = packet[DRAT_HEADER_LEN + 7];
        // seq1 may or may not be present depending on the packet type
        const uint8_t seq1 = packet[DRAT_HEADER_LEN + 6];
        const uint64_t timestamp =
            be64toh(*reinterpret_cast<uint64_t*>(&packet[DRAT_TIMESTAMP_OFFSET]));
        const double timestamp_s = static_cast<double>(timestamp) / d_timestamp_samp_rate;
        const double t_unix = static_cast<double>(d_timestamp_epoch_int) +
                              d_timestamp_epoch_frac + timestamp_s;
        switch (type) {
        case status_type::ACK:
            this->d_logger->debug("received ACK for seq {} (t = {})", seq0, t_unix);
            break;
        case status_type::EOB_ACK:
            this->d_logger->debug("received EOB_ACK for seq {} (t = {})", seq0, t_unix);
            break;
        case status_type::UNDERFLOW:
            this->d_logger->error("received UNDERFLOW (t = {})", t_unix);
            break;
        case status_type::SEQ_ERROR_START:
            this->d_logger->error(
                "received SEQ_ERROR_START; expected {}, in error {} (t = {})",
                seq1,
                seq0,
                t_unix);
            break;
        case status_type::SEQ_ERROR_MID:
            this->d_logger->error(
                "received SEQ_ERROR_MID; expected {}, in error {} (t = {})",
                seq1,
                seq0,
                t_unix);
            break;
        case status_type::LATE:
            this->d_logger->error("received LATE for seq {} (t = {})", seq0, t_unix);
            break;
        default:
            this->d_logger->error(
                "received unexpected status {} (t = {})", static_cast<int>(type), t_unix);
        }
    }
}

template <class T>
int udp_sink_impl<T>::bind_socket(const std::string& addr,
                                  int port,
                                  const std::string& name)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        throw std::runtime_error(fmt::format("could not create {} UDP socket", name));
    }

    const struct timeval timeout = { 0, 100000 }; // 100 ms
    if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        this->d_logger->error(fmt::format("could not set {} receive timeout", name));
    }

    struct sockaddr_in saddr;
    std::memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr(addr.c_str());
    saddr.sin_port = htons(port);

    const bool is_multicast = (ntohl(saddr.sin_addr.s_addr) >> 28) == 0b1110;
    if (is_multicast) {
        const int enable = 1;
        if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
            std::perror(
                fmt::format("could not set SO_REUSEADDR on {} socket", name).c_str());
            throw std::runtime_error(
                fmt::format("unable to set SO_REUSADDR on {} socket", name));
        }
    }

    if (bind(s, reinterpret_cast<const struct sockaddr*>(&saddr), sizeof(saddr)) < 0) {
        std::perror(fmt::format("could not bind {} UDP socket", name).c_str());
        throw std::runtime_error(fmt::format("unable to bind {} UDP socket", name));
    }

    if (is_multicast) {
        // join multicast group
        struct ip_mreq mreq = { saddr.sin_addr.s_addr, htonl(INADDR_ANY) };
        if (setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
            std::perror(fmt::format("could not join {} multicast group", name).c_str());
            throw std::runtime_error(
                fmt::format("unable to join {} multicast group", name));
        }
    }

    return s;
}

template <class T>
ssize_t udp_sink_impl<T>::recv_drat_packet(int socket,
                                           uint8_t* buffer,
                                           size_t buffer_len,
                                           int64_t flow_id_filter)
{
    const auto len = recvfrom(socket, buffer, buffer_len, 0, nullptr, nullptr);
    if (len < 0) {
        if (errno == EWOULDBLOCK) {
            // receive timeout expired
            return -1;
        }
        std::perror("could not receive from time report UDP socket");
        throw std::runtime_error("unable to receive from time report UDP socket");
    }

    if (len < DRAT_HEADER_LEN) {
        this->d_logger->error("received UDP packet shorter than DRaT header");
        // Drop packet
        return -1;
    }

    const uint16_t size = ntohs(*reinterpret_cast<uint16_t*>(&buffer[DRAT_SIZE_OFFSET]));
    if (size != len) {
        this->d_logger->error(
            "wrong DRaT size field (contains {}, UDP payload length {})", size, len);
        return -1;
    }

    const uint32_t flow_id =
        ntohl(*reinterpret_cast<uint32_t*>(&buffer[DRAT_FLOW_ID_OFFSET]));
    if ((flow_id_filter >= 0) && (flow_id_filter != flow_id)) {
        // Flow ID does not match. Drop the packet.
        return -1;
    }

    return len;
}

template <class T>
int udp_sink_impl<T>::get_next_eob_index(int nitems, int last_eob_index)
{
    int eob_index = nitems; // This is a value that gets never reached in the for loop

    // Examine EOB tags
    std::vector<gr::tag_t> tags;
    this->get_tags_in_range(tags,
                            0,
                            this->nitems_read(0) + last_eob_index + 1,
                            this->nitems_read(0) + nitems,
                            eob_key());
    for (const auto& tag : tags) {
        int64_t idx = tag.offset - this->nitems_read(0);
        if (idx < eob_index) {
            eob_index = idx;
        }
    }

    return eob_index;
}

template <>
void udp_sink_impl<gr_complex>::pack_samples(std::complex<int16_t>* output_vector,
                                             const gr_complex* input_vector,
                                             size_t num_samples)
{
    volk_32f_s32f_convert_16i(reinterpret_cast<int16_t*>(output_vector),
                              reinterpret_cast<const float*>(input_vector),
                              32768.0,
                              2 * num_samples);
#if DRAT_BYTE_ORDER == DRAT_LITTLE_ENDIAN
    volk_16u_byteswap(reinterpret_cast<uint16_t*>(output_vector), 2 * num_samples);
#endif
}

template <>
void udp_sink_impl<std::complex<int16_t>>::pack_samples(
    std::complex<int16_t>* output_vector,
    const std::complex<int16_t>* input_vector,
    size_t num_samples)
{
    std::memcpy(output_vector, input_vector, sizeof(std::complex<int16_t>) * num_samples);
#if DRAT_BYTE_ORDER == DRAT_LITTLE_ENDIAN
    volk_16u_byteswap(reinterpret_cast<uint16_t*>(output_vector), 2 * num_samples);
#endif
}

template <>
void udp_sink_impl<float>::pack_samples(float* output_vector,
                                        const float* input_vector,
                                        size_t num_samples)
{
    // the samples in the DRaT packet are float32 little-endian
    std::memcpy(output_vector, input_vector, sizeof(float) * num_samples);
#if DRAT_BYTE_ORDER == DRAT_BIG_ENDIAN
    volk_32u_byteswap(reinterpret_cast<uint32_t*>(output_vector), num_samples);
#endif
}

template <>
void udp_sink_impl<uint8_t>::pack_samples(uint8_t* output_vector,
                                          const uint8_t* input_vector,
                                          size_t num_samples)
{
    std::memcpy(output_vector, input_vector, num_samples);
}

template class udp_sink<gr_complex>;
template class udp_sink<std::complex<int16_t>>;
template class udp_sink<float>;
template class udp_sink<uint8_t>;

} /* namespace drat */
} /* namespace gr */
