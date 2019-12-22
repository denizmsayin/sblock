// this is simply an external interface for the DaviInputGenerator class
// its functions are wrapped and exported C-style for use by python's ctypes
// __H & __W should be changed and different libraries compiled for different puzzle sizes
// the build system handles this automatically

#include "davi_input_generator.hpp"

#ifndef __H
#define __H 3
#endif
#ifndef __W
#define __W 3
#endif

constexpr sbpuzzle::psize_t H = __H, W = __W;

using DIG = sbpuzzle::DaviInputGenerator<H, W>;

extern "C" {
    DIG *DIG_new(unsigned seed, size_t batch_size) {
        return new DIG(seed, batch_size);
    }

    void DIG_delete(DIG *dig) {
        delete dig;
    }

    void DIG_init_batch(DIG *dig, int64_t smin, int64_t smax) {
        dig->init_batch(smin, smax);
    }

    void DIG_get_batch_states(DIG *dig, void *out, bool *out_is_goal) {
        dig->get_batch_states(out, out_is_goal);
    }

    bool DIG_has_more_neighbors(DIG *dig) {
        return dig->has_more_neighbors();
    }

    void DIG_get_next_neighbors(DIG *dig,
                                void *out,
                                bool *out_exists,
                                bool *out_is_goal)
    {
        dig->get_next_neighbors(out, out_exists, out_is_goal);
    }
}
