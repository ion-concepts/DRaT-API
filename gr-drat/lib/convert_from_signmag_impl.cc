/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "convert_from_signmag_impl.h"
#include <gnuradio/io_signature.h>
#include <type_traits>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace gr {
namespace drat {

template <typename T, const bool MSB_FIRST>
typename convert_from_signmag<T, MSB_FIRST>::sptr
convert_from_signmag<T, MSB_FIRST>::make(size_t vlen_in, size_t vlen_out, float scale)
{
    return gnuradio::make_block_sptr<convert_from_signmag_impl<T, MSB_FIRST>>(
        vlen_in, vlen_out, scale);
}

template <typename T, const bool MSB_FIRST>
convert_from_signmag_impl<T, MSB_FIRST>::convert_from_signmag_impl(size_t vlen_in,
                                                                   size_t vlen_out,
                                                                   float scale)
    : gr::sync_interpolator("convert_from_signmag",
                            gr::io_signature::make(1, 1, sizeof(int16_t) * vlen_in),
                            gr::io_signature::make(1, 1, sizeof(T) * vlen_out),
                            (SAMPLES_PER_I16 * vlen_in) / vlen_out),
      d_points_per_item(vlen_out),
      d_scale(scale)
{
    if ((SAMPLES_PER_I16 * vlen_in) % vlen_out != 0) {
        throw std::out_of_range(
            "[convert_from_signmag] the number of samples per output item must be "
            "divisible by the number of samples per input item");
    }
}

template <typename T, const bool MSB_FIRST>
convert_from_signmag_impl<T, MSB_FIRST>::~convert_from_signmag_impl()
{
}

// The implementation of this template function could be replaced by calls
// to appropriate Volk kernels (which do not exist yet).
template <typename T, const bool MSB_FIRST>
static void signmag_s32f_convert_T(T* out,
                                   const uint8_t* in,
                                   const float scale,
                                   unsigned int num_points)
{
    static_assert(std::is_integral<T>() || std::is_floating_point<T>());
    const float scale_inv = 1.0f / scale;
    while (num_points >= 8) {
        uint8_t signs;
        uint8_t mags;
        if constexpr (MSB_FIRST) {
            mags = *in++;
            signs = *in++;
        } else {
            signs = *in++;
            mags = *in++;
        }
        // The XOR with the signs is because this sign-magnitude
        // representation works like 2's complement instead of 1's
        // complement (see convert_to_signmag.h for details).
        mags ^= signs;

        for (int j = 0; j < 8; ++j) {
            const uint8_t sign = (signs >> (7 - j)) & 1;
            const uint8_t mag = ((mags >> (7 - j)) & 1);
            int signmag = 1 + 2 * mag;
            if (sign) {
                signmag = -signmag;
            }
            const float scaled = static_cast<float>(signmag) * scale_inv;
            T converted;
            if constexpr (std::is_integral<T>()) {
                // integral type: round and clamp
                converted = static_cast<T>(
                    std::clamp(std::round(scaled),
                               static_cast<float>(std::numeric_limits<T>::min()),
                               static_cast<float>(std::numeric_limits<T>::max())));
            } else {
                // floating point type: no need to round or clamp
                converted = static_cast<T>(scaled);
            }
            *out++ = converted;
        }

        num_points -= 8;
    }
}

template <typename T, const bool MSB_FIRST>
int convert_from_signmag_impl<T, MSB_FIRST>::work(int noutput_items,
                                                  gr_vector_const_void_star& input_items,
                                                  gr_vector_void_star& output_items)
{
    auto in = static_cast<const uint8_t*>(input_items[0]);
    auto out = static_cast<T*>(output_items[0]);

    signmag_s32f_convert_T<T, MSB_FIRST>(
        out, in, d_scale, noutput_items * d_points_per_item);

    return noutput_items;
}

template class convert_from_signmag<float>;
template class convert_from_signmag<int32_t>;
template class convert_from_signmag<int16_t>;
template class convert_from_signmag<int8_t>;
template class convert_from_signmag<float, true>;
template class convert_from_signmag<int32_t, true>;
template class convert_from_signmag<int16_t, true>;
template class convert_from_signmag<int8_t, true>;

} /* namespace drat */
} /* namespace gr */
