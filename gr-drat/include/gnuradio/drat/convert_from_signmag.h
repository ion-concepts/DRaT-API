/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DRAT_CONVERT_FROM_SIGNMAG_H
#define INCLUDED_DRAT_CONVERT_FROM_SIGNMAG_H

#include <gnuradio/drat/api.h>
#include <gnuradio/sync_interpolator.h>

namespace gr {
namespace drat {

/*!
 * \brief Convert from Sign-Magnitude
 * \ingroup drat
 *
 * This block reverts the conversion to Sign-Magnitude format done by
 * convert_to_signmag. See the documentation for convert_to_signmag for details.
 */
template <typename T, const bool MSB_FIRST = false>
class DRAT_API convert_from_signmag : virtual public gr::sync_interpolator
{
public:
    typedef std::shared_ptr<convert_from_signmag> sptr;

    /*!
     * \brief Create a Convert from Sign-Magnitude block.
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

typedef convert_from_signmag<float> convert_from_signmag_sfF;
typedef convert_from_signmag<int32_t> convert_from_signmag_siF;
typedef convert_from_signmag<int16_t> convert_from_signmag_ssF;
typedef convert_from_signmag<int8_t> convert_from_signmag_sbF;
typedef convert_from_signmag<float, true> convert_from_signmag_sfT;
typedef convert_from_signmag<int32_t, true> convert_from_signmag_siT;
typedef convert_from_signmag<int16_t, true> convert_from_signmag_ssT;
typedef convert_from_signmag<int8_t, true> convert_from_signmag_sbT;

} // namespace drat
} // namespace gr

#endif /* INCLUDED_DRAT_CONVERT_FROM_SIGNMAG_H */
