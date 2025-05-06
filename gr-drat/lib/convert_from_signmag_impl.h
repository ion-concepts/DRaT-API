/* -*- c++ -*- */
/*
 * Copyright 2023 gr-drat author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_DRAT_CONVERT_FROM_SIGNMAG_IMPL_H
#define INCLUDED_DRAT_CONVERT_FROM_SIGNMAG_IMPL_H

#include <gnuradio/drat/convert_from_signmag.h>

namespace gr {
namespace drat {

template <typename T, const bool MSB_FIRST>
class convert_from_signmag_impl : public convert_from_signmag<T, MSB_FIRST>
{
private:
    const int d_points_per_item;
    float d_scale;

    static constexpr int SAMPLES_PER_I16 = 8;

public:
    convert_from_signmag_impl(size_t vlen_in, size_t vlen_out, float scale);
    ~convert_from_signmag_impl() override;

    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items) override;

    float scale() const override { return d_scale; }

    void set_scale(float scale) override { d_scale = scale; }
};

} // namespace drat
} // namespace gr

#endif /* INCLUDED_DRAT_CONVERT_FROM_SIGNMAG_IMPL_H */
