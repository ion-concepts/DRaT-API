/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DRAT_CONVERT_TO_SIGNMAG_H
#define INCLUDED_DRAT_CONVERT_TO_SIGNMAG_H

#include <gnuradio/drat/api.h>
#include <gnuradio/sync_decimator.h>

namespace gr {
namespace drat {

/*!
 * \brief Convert to Sign-Magnitude
 * \ingroup drat
 *
 * This block converts the input items to a 2-bit sign-mangitude format which
 * only represents the integers -3, -1, +1, and +3. The 2-bit sign-magnitude
 * items are packed into an int16_t. By default the 8 signs are packed into the
 * first byte in memory, and the 8 magnitudes packed into the second byte in
 * memory (this can be changed with the MSB_FIRST template parameter). Within
 * each byte, the sign or magnitude for the first sample is packed into the most
 * significant bit.
 *
 * The representation of the values -3, -1, +1 and +3 as sign-magnitude is as
 * follows (it resembles a 2's complement representation more than a
 * 1's complement representation).
 *
 * Value | Sign bit | Magnitude bit
 * --------------------------------
 *    -3 |        1 |            0
 *    -1 |        1 |            1
 *    +1 |        0 |            0
 *    +3 |        0 |            1
 *
 * The scale parameter of this block is consistent with how the scale works for
 * in-tree conversion blocks. The input is multiplied by the scale, and the
 * result is rounded to the closest representable integer (-3, -1, +1, or
 * +3). Therefore, the magnitude bit will be set for those inputs whose absolute
 * value is larger than 2.0 / scale.
 */
template <typename T, const bool MSB_FIRST = false>
class DRAT_API convert_to_signmag : virtual public gr::sync_decimator
{
public:
    typedef std::shared_ptr<convert_to_signmag> sptr;

    /*!
     * \brief Create a Convert to Sign-Magnitude block.
     *
     * \param vlen_in Vector length of the input.
     * \param vlen_out Vector length of the ouput.
     * \param scale Scale factor for the conversion.
     */
    static sptr make(size_t vlen_in = 1, size_t vlen_out = 1, float scale = 1.0);

    /* \brief Returns the scale factor */
    virtual float scale() const = 0;

    /* \brief Sets the scale factor */
    virtual void set_scale(float scale) = 0;
};

typedef convert_to_signmag<float> convert_to_signmag_fsF;
typedef convert_to_signmag<int32_t> convert_to_signmag_isF;
typedef convert_to_signmag<int16_t> convert_to_signmag_ssF;
typedef convert_to_signmag<int8_t> convert_to_signmag_bsF;
typedef convert_to_signmag<float, true> convert_to_signmag_fsT;
typedef convert_to_signmag<int32_t, true> convert_to_signmag_isT;
typedef convert_to_signmag<int16_t, true> convert_to_signmag_ssT;
typedef convert_to_signmag<int8_t, true> convert_to_signmag_bsT;

} // namespace drat
} // namespace gr

#endif /* INCLUDED_DRAT_CONVERT_TO_SIGNMAG_H */
