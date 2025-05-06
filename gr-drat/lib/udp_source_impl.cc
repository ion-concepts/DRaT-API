/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "drat.h"
#include "udp_source_impl.h"
#include <gnuradio/io_signature.h>
#include <arpa/inet.h>
#include <cmake_endian.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace gr {
namespace drat {

const pmt::pmt_t lost_packets_key()
{
    static const auto key = pmt::string_to_symbol("lost_packets");
    return key;
}

const pmt::pmt_t num_lost_key()
{
    static const auto key = pmt::string_to_symbol("num_lost");
    return key;
}

const pmt::pmt_t new_seq_key()
{
    static const auto key = pmt::string_to_symbol("new_seq");
    return key;
}

const pmt::pmt_t expected_seq_key()
{
    static const auto key = pmt::string_to_symbol("expected_seq");
    return key;
}

const pmt::pmt_t total_packets_key()
{
    static const auto key = pmt::string_to_symbol("total_packets");
    return key;
}

const pmt::pmt_t sob_key()
{
    static const auto key = pmt::string_to_symbol("SOB");
    return key;
}

const pmt::pmt_t eob_key()
{
    static const auto key = pmt::string_to_symbol("EOB");
    return key;
}

const pmt::pmt_t rx_time_key()
{
    static const auto key = pmt::string_to_symbol("rx_time");
    return key;
}

const pmt::pmt_t drat_time_key()
{
    static const auto key = pmt::string_to_symbol("drat_time");
    return key;
}

const pmt::pmt_t timestamps_port()
{
    static const auto key = pmt::string_to_symbol("timestamps");
    return key;
}

const pmt::pmt_t lost_packets_port()
{
    static const auto key = pmt::string_to_symbol("lost_packets");
    return key;
}

template <class T>
typename udp_source<T>::sptr udp_source<T>::make(const std::string& listen_addr,
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
                                                 int custom_drat_type_eob)
{
    return gnuradio::make_block_sptr<udp_source_impl<T>>(listen_addr,
                                                         listen_port,
                                                         samp_rate,
                                                         timestamp_epoch,
                                                         flow_id,
                                                         burst_mode,
                                                         status_addr,
                                                         status_port,
                                                         status_flow_id,
                                                         send_ack_every,
                                                         zero_fill_lost,
                                                         ticks_per_sample,
                                                         recv_timeout,
                                                         custom_drat_type,
                                                         custom_drat_type_eob);
}


template <class T>
udp_source_impl<T>::udp_source_impl(const std::string& listen_addr,
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
                                    int custom_drat_type_eob)
    : gr::sync_block("udp_source",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, 1, sizeof(T))),
      d_packet_buffer(DRAT_MAX_MTU, 0),
      d_curr_pointer(d_packet_buffer.data()),
      d_end_pointer(d_packet_buffer.data()),
      d_samp_rate(samp_rate),
      d_samp_rate_int(static_cast<uint64_t>(samp_rate)),
      d_samp_rate_frac(samp_rate - static_cast<double>(d_samp_rate_int)),
      d_timestamp_epoch_int(static_cast<uint64_t>(timestamp_epoch)),
      d_timestamp_epoch_frac(timestamp_epoch -
                             static_cast<double>(d_timestamp_epoch_int)),
      d_flow_id(flow_id),
      d_burst_mode(burst_mode),
      d_eob(true),
      d_last_seq(0),
      d_expected_timestamp(0),
      d_total_packets(0),
      d_status_seq(0),
      d_status_flow_id(status_flow_id),
      d_send_ack_every(send_ack_every),
      d_zero_fill_lost(zero_fill_lost),
      d_to_zero_fill(0),
      d_ticks_per_sample(ticks_per_sample),
      d_drat_type(custom_drat_type >= 0 ? std::optional<uint8_t>{ custom_drat_type }
                                        : std::nullopt),
      d_drat_type_eob(custom_drat_type_eob >= 0
                          ? std::optional<uint8_t>{ custom_drat_type_eob }
                          : std::nullopt)
{
    this->message_port_register_out(timestamps_port());
    this->message_port_register_out(lost_packets_port());

    d_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (d_socket < 0) {
        throw std::runtime_error("could not create UDP socket");
    }

    if (recv_timeout >= 0.0) {
        const int recv_timeout_s = recv_timeout / 1000.0;
        struct timeval timeout = {
            recv_timeout_s,
            static_cast<long>(round(1000.0 * (recv_timeout - 1000.0 * recv_timeout_s)))
        };
        if (setsockopt(d_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) <
            0) {
            this->d_logger->error("could not set socket receive timeout");
        }
    }

    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(listen_addr.c_str());
    server_addr.sin_port = htons(listen_port);

    const bool is_multicast = (ntohl(server_addr.sin_addr.s_addr) >> 28) == 0b1110;
    if (is_multicast) {
        const int enable = 1;
        if (setsockopt(d_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
            std::perror("could not set SO_REUSEADDR");
            throw std::runtime_error("unable to set SO_REUSADDR");
        }
    }

    if (bind(d_socket, (const struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::perror("could not bind UDP socket");
        throw std::runtime_error("unable to bind UDP socket");
    }

    if (is_multicast) {
        // join multicast group
        struct ip_mreq mreq = { server_addr.sin_addr.s_addr, htonl(INADDR_ANY) };
        if (setsockopt(d_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) <
            0) {
            std::perror("could not join multicast group");
            throw std::runtime_error("unable to join multicast group");
        }
    }

    if (!status_addr.empty()) {
        d_status_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (d_status_socket < 0) {
            throw std::runtime_error("could not create status UDP socket");
        }

        struct sockaddr_in address;
        std::memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = inet_addr(status_addr.c_str());
        address.sin_port = htons(status_port);
        if (connect(d_status_socket,
                    reinterpret_cast<const struct sockaddr*>(&address),
                    sizeof(address)) < 0) {
            std::perror("could not connect status socket");
            throw std::runtime_error("unable to connect status socket");
        }
    } else {
        d_status_socket = -1;
    }
}

template <class T>
udp_source_impl<T>::~udp_source_impl()
{
    close(d_socket);
    if (d_status_socket >= 0) {
        close(d_status_socket);
    }
}

template <class T>
int udp_source_impl<T>::work(int noutput_items,
                             gr_vector_const_void_star& input_items,
                             gr_vector_void_star& output_items)
{
    auto out = static_cast<T*>(output_items[0]);
    const auto out_end = out + noutput_items;
    int produced = 0;

    while (out < out_end) {
        if (d_to_zero_fill > 0) {
            // We have lost some packets and are in zero-fill mode, so we need
            // to produce zeros on the output to replace the lost samples.
            const size_t can_produce = out_end - out;
            const auto nitems = std::min(can_produce, d_to_zero_fill);
            std::memset(reinterpret_cast<uint8_t*>(out), 0, nitems * sizeof(T));
            out += nitems;
            produced += nitems;
            d_to_zero_fill -= nitems;
            continue;
        }

        if (d_curr_pointer == d_end_pointer) {
            // UDP packet finished. Receive a new one
            const auto len = recvfrom(d_socket,
                                      d_packet_buffer.data(),
                                      d_packet_buffer.size(),
                                      0,
                                      nullptr,
                                      nullptr);
            if (len < 0) {
                if (errno == EWOULDBLOCK) {
                    // receive timeout expired
                    break;
                }
                std::perror("could not receive from UDP socket");
                throw std::runtime_error("unable to receive from UDP socket");
            }

            if (len < DRAT_HEADER_LEN) {
                this->d_logger->error("received UDP packet shorter than DRaT header");
                // Drop packet and receive a new one
                continue;
            }

            const uint16_t size =
                ntohs(*reinterpret_cast<uint16_t*>(&d_packet_buffer[DRAT_SIZE_OFFSET]));
            if (size != len) {
                this->d_logger->error(
                    "wrong DRaT size field (contains {}, UDP payload length {})",
                    size,
                    len);
                continue;
            }

            const uint32_t flow_id = ntohl(
                *reinterpret_cast<uint32_t*>(&d_packet_buffer[DRAT_FLOW_ID_OFFSET]));
            if ((d_flow_id >= 0) && (d_flow_id != flow_id)) {
                // Flow ID does not match. Drop the packet.
                continue;
            }

            const auto prev_eob = d_eob;
            bool async = false;
            const uint8_t type_u8 = d_packet_buffer[DRAT_TYPE_OFFSET];
            const packet_type type = packet_type(type_u8);
            if ((d_drat_type.has_value() && (type_u8 == *d_drat_type)) ||
                type == packet_type::INT16_COMPLEX ||
                type == packet_type::INT16_COMPLEX_ASYNC) {
                d_eob = false;
            } else if ((d_drat_type_eob.has_value() && (type_u8 == *d_drat_type_eob)) ||
                       type == packet_type::INT16_COMPLEX_EOB ||
                       type == packet_type::INT16_COMPLEX_ASYNC_EOB) {
                d_eob = true;
                async = true;
            } else if (type == packet_type::TIME_REPORT) {
                // silently ignore time reports
                continue;
            } else {
                this->d_logger->error("unexpected DRaT packet type {}; dropping",
                                      static_cast<int>(type));
                continue;
            }

            ++d_total_packets;

            // The timestamp of 0 in async mode is only used to send status packets
            const uint64_t timestamp = !async
                                           ? be64toh(*reinterpret_cast<uint64_t*>(
                                                 &d_packet_buffer[DRAT_TIMESTAMP_OFFSET]))
                                           : 0;
            const uint8_t seq = d_packet_buffer[DRAT_SEQ_OFFSET];
            const uint8_t expected_seq = d_last_seq + 1;
            const uint8_t num_lost = seq - expected_seq;
            if ((num_lost != 0) && (d_total_packets > 1)) {
                this->d_logger->warn("sequence error: got {}, expected {} (lost {} "
                                     "modulo 256), packets received {}",
                                     seq,
                                     expected_seq,
                                     num_lost,
                                     d_total_packets);
                auto lost = pmt::make_dict();
                lost = pmt::dict_add(lost, num_lost_key(), pmt::from_long(num_lost));
                lost = pmt::dict_add(lost, new_seq_key(), pmt::from_long(seq));
                lost =
                    pmt::dict_add(lost, expected_seq_key(), pmt::from_long(expected_seq));
                lost = pmt::dict_add(
                    lost, total_packets_key(), pmt::from_uint64(d_total_packets));
                this->add_item_tag(
                    0, this->nitems_written(0) + produced, lost_packets_key(), lost);
                this->message_port_pub(lost_packets_port(), lost);

                send_status(prev_eob ? status_type::SEQ_ERROR_START
                                     : status_type::SEQ_ERROR_MID,
                            timestamp,
                            seq,
                            expected_seq);
            }
            // After EOB, the next packet should have sequence 0, so we pretend
            // that the EOB packet had sequence 255 in order to use the same
            // check logic.
            d_last_seq = d_eob ? 255 : seq;

            if (d_eob) {
                send_status(status_type::EOB_ACK, timestamp, seq);
            } else if ((d_send_ack_every > 0) &&
                       (d_total_packets % d_send_ack_every == 0)) {
                send_status(status_type::ACK, timestamp, seq);
            }

            if (d_burst_mode && prev_eob) {
                // First sample in the burst. Insert SOB tag.
                this->add_item_tag(
                    0, this->nitems_written(0) + produced, sob_key(), pmt::PMT_NIL);
            }

            if ((prev_eob || num_lost > 0) && !async) {
                if (!prev_eob && d_zero_fill_lost) {
                    // Samples have been lost mid-burst. Arrange so that we
                    // produce zeros to replace the lost samples
                    const uint64_t ticks_lost = timestamp - d_expected_timestamp;
                    if (ticks_lost % d_ticks_per_sample != 0) {
                        this->d_logger->warn(
                            "lost {} timestamp ticks, which is not a multiple of ticks "
                            "per sample {}; cannot zero-fill exactly the lost samples",
                            ticks_lost,
                            d_ticks_per_sample);
                    }
                    d_to_zero_fill = ticks_lost / d_ticks_per_sample;
                    this->d_logger->warn("zero-filling {} lost samples", d_to_zero_fill);
                }

                // Insert timestamp tag in output and send message.
                const double timestamp_s = static_cast<double>(timestamp) / d_samp_rate;
                const uint64_t timestamp_s_int = static_cast<uint64_t>(timestamp_s);
                const uint64_t bulk_ticks = timestamp_s_int * d_samp_rate_int;
                const uint64_t remain_ticks = timestamp - bulk_ticks;
                const double frac_ticks =
                    static_cast<double>(remain_ticks) -
                    static_cast<double>(timestamp_s_int) * d_samp_rate_frac;
                const double timestamp_s_frac = frac_ticks / d_samp_rate;
                uint64_t unix_int = d_timestamp_epoch_int + timestamp_s_int;
                double unix_frac = d_timestamp_epoch_frac + timestamp_s_frac;
                const int64_t carry = static_cast<int64_t>(unix_frac);
                unix_int += carry;
                unix_frac += static_cast<double>(carry);

                const auto timestamp_pmt = pmt::make_tuple(pmt::from_uint64(unix_int),
                                                           pmt::from_double(unix_frac));
                // The timestamp is assigned to the beginning of the samples of
                // this packet, so d_to_zero_fill is taken into account for its
                // offset.
                const auto nitems = this->nitems_written(0) + produced + d_to_zero_fill;
                this->add_item_tag(
                    0, nitems, drat_time_key(), pmt::from_uint64(timestamp));
                this->add_item_tag(0, nitems, rx_time_key(), timestamp_pmt);
                this->message_port_pub(timestamps_port(), timestamp_pmt);
            }

            if (!async && d_zero_fill_lost) {
                const auto num_samples =
                    (len - DRAT_HEADER_LEN) / sizeof(drat_packing_t<T>);
                d_expected_timestamp = timestamp + num_samples * d_ticks_per_sample;
            }

            d_curr_pointer = &d_packet_buffer[DRAT_HEADER_LEN];
            d_end_pointer = &d_packet_buffer[len];

            if (d_to_zero_fill > 0) {
                // Skip processing this packet for now, since we need to zero
                // fill to account for lost packets.
                continue;
            }
        }

        const size_t can_produce = out_end - out;
        const size_t can_consume =
            (d_end_pointer - d_curr_pointer) / sizeof(drat_packing_t<T>);
        const auto nitems = std::min(can_produce, can_consume);
        unpack_samples(out, reinterpret_cast<drat_packing_t<T>*>(d_curr_pointer), nitems);
        out += nitems;
        produced += nitems;
        d_curr_pointer += nitems * sizeof(drat_packing_t<T>);

        if (d_eob) {
            if (d_burst_mode && (d_curr_pointer == d_end_pointer)) {
                // Insert EOB tag in last sample of burst.
                this->add_item_tag(
                    0, this->nitems_written(0) + produced - 1, eob_key(), pmt::PMT_NIL);
            }
            // We are at the end of a burst. Return to the scheduler instead of
            // waiting to receive more packets to fill the output buffer.
            break;
        }
    }

    return produced;
}

template <class T>
void udp_source_impl<T>::send_status(status_type type,
                                     uint64_t timestamp,
                                     uint8_t seq0,
                                     uint8_t seq1)
{
    if (d_status_socket < 0) {
        // status sending disabled
        return;
    }

    uint8_t packet[24];
    packet[DRAT_TYPE_OFFSET] = static_cast<uint8_t>(packet_type::STATUS);
    packet[DRAT_SEQ_OFFSET] = ++d_status_seq;
    *reinterpret_cast<uint16_t*>(&packet[DRAT_SIZE_OFFSET]) = htons(24);
    *reinterpret_cast<uint32_t*>(&packet[DRAT_FLOW_ID_OFFSET]) = htonl(d_status_flow_id);
    *reinterpret_cast<uint64_t*>(&packet[DRAT_TIMESTAMP_OFFSET]) = htobe64(timestamp);
    *reinterpret_cast<uint32_t*>(&packet[DRAT_HEADER_LEN]) =
        htonl(static_cast<uint32_t>(type));
    // unused field
    *reinterpret_cast<uint16_t*>(&packet[DRAT_HEADER_LEN + 4]) = 0;
    packet[DRAT_HEADER_LEN + 6] = seq1;
    packet[DRAT_HEADER_LEN + 7] = seq0;

    if (send(d_status_socket, packet, sizeof(packet), 0) < 0) {
        // This can happen if the remote end is sending ICMP destination
        // unreachable (port unreachable), so we simply print an error
        // but do not fail.
        std::perror("[DRaT UDP Source] could not send status to UDP socket");
    }
}

template <>
void udp_source_impl<gr_complex>::unpack_samples(gr_complex* output_vector,
                                                 std::complex<int16_t>* input_vector,
                                                 size_t num_samples)
{
#if DRAT_BYTE_ORDER == DRAT_LITTLE_ENDIAN
    volk_16u_byteswap(reinterpret_cast<uint16_t*>(input_vector), 2 * num_samples);
#endif
    volk_16i_s32f_convert_32f(reinterpret_cast<float*>(output_vector),
                              reinterpret_cast<const int16_t*>(input_vector),
                              32768.0,
                              2 * num_samples);
}

template <>
void udp_source_impl<std::complex<int16_t>>::unpack_samples(
    std::complex<int16_t>* output_vector,
    std::complex<int16_t>* input_vector,
    size_t num_samples)
{
#if DRAT_BYTE_ORDER == DRAT_LITTLE_ENDIAN
    volk_16u_byteswap(reinterpret_cast<uint16_t*>(input_vector), 2 * num_samples);
#endif
    std::memcpy(output_vector, input_vector, sizeof(std::complex<int16_t>) * num_samples);
}

template <>
void udp_source_impl<float>::unpack_samples(float* output_vector,
                                            float* input_vector,
                                            size_t num_samples)
{
    // the samples in the DRaT packet are float32 little-endian
#if DRAT_BYTE_ORDER == DRAT_BIG_ENDIAN
    volk_32u_byteswap(reinterpret_cast<uint32_t*>(input_vector), num_samples);
#endif
    std::memcpy(output_vector, input_vector, sizeof(float) * num_samples);
}

template <>
void udp_source_impl<uint8_t>::unpack_samples(uint8_t* output_vector,
                                              uint8_t* input_vector,
                                              size_t num_samples)
{
    std::memcpy(output_vector, input_vector, num_samples);
}

template class udp_source<gr_complex>;
template class udp_source<std::complex<int16_t>>;
template class udp_source<float>;
template class udp_source<uint8_t>;

} /* namespace drat */
} /* namespace gr */
