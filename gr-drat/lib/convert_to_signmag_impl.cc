/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "convert_to_signmag_impl.h"
#include <gnuradio/io_signature.h>
#include <cstdint>
#include <stdexcept>

namespace gr {
namespace drat {

template <typename T, const bool MSB_FIRST>
typename convert_to_signmag<T, MSB_FIRST>::sptr
convert_to_signmag<T, MSB_FIRST>::make(size_t vlen_in, size_t vlen_out, float scale)
{
    return gnuradio::make_block_sptr<convert_to_signmag_impl<T, MSB_FIRST>>(
        vlen_in, vlen_out, scale);
}

template <typename T, const bool MSB_FIRST>
convert_to_signmag_impl<T, MSB_FIRST>::convert_to_signmag_impl(size_t vlen_in,
                                                               size_t vlen_out,
                                                               float scale)
    : gr::sync_decimator("convert_to_signmag",
                         gr::io_signature::make(1, 1, sizeof(T) * vlen_in),
                         gr::io_signature::make(1, 1, sizeof(int16_t) * vlen_out),
                         (SAMPLES_PER_I16 * vlen_out) / vlen_in),
      d_points_per_item(SAMPLES_PER_I16 * vlen_out),
      d_scale(scale)
{
    if ((SAMPLES_PER_I16 * vlen_out) % vlen_in != 0) {
        throw std::out_of_range(
            "[convert_to_signmag] the number of samples per output item must be "
            "divisible by the number of samples per input item");
    }
}

template <typename T, const bool MSB_FIRST>
convert_to_signmag_impl<T, MSB_FIRST>::~convert_to_signmag_impl()
{
}

// The implementation of this template function could be replaced by calls
// to appropriate Volk kernels (which do not exist yet).
template <typename T, const bool MSB_FIRST>
static void T_s32f_convert_signmag(uint8_t* out,
                                   const T* in,
                                   const float scale,
                                   unsigned int num_points)
{
    const T mag_threshold = static_cast<T>(2.0f / scale);
    while (num_points >= 8) {
        uint8_t signs = 0;
        uint8_t mags = 0;

        for (int j = 0; j < 8; ++j) {
            const T a = *in++;
            constexpr T ZERO = static_cast<T>(0);
            const uint8_t sign = a < ZERO;
            const uint8_t mag = (std::abs(a) >= mag_threshold);
            signs = (signs << 1) | sign;
            mags = (mags << 1) | mag;
        }
        // The XOR with the signs is because this sign-magnitude
        // representation works like 2's complement instead of 1's
        // complement (see convert_to_signmag.h for details).
        mags ^= signs;

        if constexpr (MSB_FIRST) {
            *out++ = mags;
            *out++ = signs;
        } else {
            *out++ = signs;
            *out++ = mags;
        }
        num_points -= 8;
    }
}

template <typename T, const bool MSB_FIRST>
int convert_to_signmag_impl<T, MSB_FIRST>::work(int noutput_items,
                                                gr_vector_const_void_star& input_items,
                                                gr_vector_void_star& output_items)
{
    auto in = static_cast<const T*>(input_items[0]);
    auto out = static_cast<uint8_t*>(output_items[0]);

    T_s32f_convert_signmag<T, MSB_FIRST>(
        out, in, d_scale, noutput_items * d_points_per_item);

    return noutput_items;
}

template class convert_to_signmag<float>;
template class convert_to_signmag<int32_t>;
template class convert_to_signmag<int16_t>;
template class convert_to_signmag<int8_t>;
template class convert_to_signmag<float, true>;
template class convert_to_signmag<int32_t, true>;
template class convert_to_signmag<int16_t, true>;
template class convert_to_signmag<int8_t, true>;

} /* namespace drat */
} /* namespace gr */
