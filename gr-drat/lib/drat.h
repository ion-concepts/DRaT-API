/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <gnuradio/gr_complex.h>
#include <complex>
#include <cstdint>

#ifndef INCLUDED_DRAT_DRAT_H
#define INCLUDED_DRAT_DRAT_H

#define DRAT_MAX_MTU 9000

#define DRAT_HEADER_LEN 16

#define DRAT_TYPE_OFFSET 0
#define DRAT_SEQ_OFFSET 1
#define DRAT_SIZE_OFFSET 2
#define DRAT_FLOW_ID_OFFSET 4
#define DRAT_TIMESTAMP_OFFSET 8

namespace gr {
namespace drat {

// https://github.com/ion-concepts/DiRT/blob/ian/ether_switching/lib/global/drat_protocol.sv

enum class packet_type {
    // Integer complex numbers in a 16bit format.
    INT16_COMPLEX = 0x00,
    // Integer complex numbers in a 16bit format. Marks end of burst.
    INT16_COMPLEX_EOB = 0x10,
    // Integer complex numbers in a 16bit format. Timestamp unused.
    INT16_COMPLEX_ASYNC = 0x20,
    // Integer complex numbers in a 16bit format. Timestamp unused. Marks end of burst.
    INT16_COMPLEX_ASYNC_EOB = 0x30,
    // Integer real numbers in a 16bit format. Used for example, for real valued sample
    // data.
    INT16_REAL = 0x01,
    // Integer complex numbers in a 12bit (packed) format. Used for example for IQ sample
    // data.
    //  INT12_COMPLEX,
    // Integer real numbers in a 12bit (packed) format. Used for example for IQ sample
    // data
    //  INT12_REAL,
    // Float complex numbers in an IEEE 32bit format. Used for example for IQ sample data
    FLOAT32_COMPLEX = 0x02,
    // Float real numbers in an IEEE 32bit format. Used for example for IQ sample data
    FLOAT32_REAL = 0x03,
    // Integer complex numbers in a 16 vectors of 16bit format.
    INT16x16_COMPLEX = 0x08,
    // Integer complex numbers in a 16 vectors of 16bit format. Marks end of burst.
    INT16x16_COMPLEX_EOB = 0x18,
    // Integer complex numbers in a 16 vectors of 16bit format. Timestamp unused.
    INT16x16_COMPLEX_ASYNC = 0x28,
    // Integer complex numbers in a 16 vectors of 16bit format. Timestamp unused. Marks
    // end of burst.
    INT16x16_COMPLEX_ASYNC_EOB = 0x38,
    // Create single 32bit memory mapped write transaction (single beat - no burst).
    WRITE_MM32 = 0x80,
    // Create single 32bit memory mapped read transaction (single beat - no burst).
    READ_MM32 = 0x81,
    // Response packet for 32bit memory mapped read transaction
    RESPONSE_MM32 = 0x86,
    // Create single 16bit memory mapped write transaction (single beat - no burst).
    WRITE_MM16 = 0x82,
    // Create single 16bit memory mapped read transaction (single beat - no burst).
    READ_MM16 = 0x83,
    // Response packet for 16bit memory mapped read transaction
    RESPONSE_MM16 = 0x87,
    // Create single 8bit memory mapped write transaction (single beat - no burst).
    WRITE_MM8 = 0x84,
    // Create single 8bit memory mapped read transaction (single beat - no burst).
    READ_MM8 = 0x85,
    // Response packet for 8bit memory mapped read transaction
    RESPONSE_MM8 = 0x88,
    // Provides "execution" status for other packets back towards host
    STATUS = 0xC0,

    // Provides a report of the current System Time
    TIME_REPORT = 0xC1,
    STRUCTURED = 0xFF
};

enum class status_type {
    ACK = 0x0,
    EOB_ACK = 0x1,
    UNDERFLOW = 0x2,
    SEQ_ERROR_START = 0x4,
    SEQ_ERROR_MID = 0x8,
    LATE = 0x10
};

// This defines how GNU Radio / CPU-native data types are mapped to DRaT / network data
// types
template <typename T>
struct drat_packing {
};

template <>
struct drat_packing<gr_complex> {
    using type = std::complex<int16_t>;
};

template <>
struct drat_packing<std::complex<int16_t>> {
    using type = std::complex<int16_t>;
};

template <>
struct drat_packing<float> {
    using type = float;
};

template <>
struct drat_packing<uint8_t> {
    using type = uint8_t;
};

template <typename T>
using drat_packing_t = typename drat_packing<T>::type;

} // namespace drat
} // namespace gr

#endif /* INCLUDED_DRAT_DRAT_H */
