#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_HEURISTICS_PDB_BASE_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_HEURISTICS_PDB_BASE_HPP

#include <array>
#include <cstdint>

#include "../../basic.hpp"

// A simple base class for different pattern database classes,
// to allow their combination via runtime polymorphism
// TODO: think about how to cleanly remove runtime polymorphism

namespace denizmsayin::sblock::sbpuzzle::heuristics::pdb {
    
    template <psize_t H, psize_t W>
    class Base {
    public:
        virtual ~Base() {}

        virtual pcost_t lookup(const std::array<pcell_t, H*W> &tiles) const = 0;

        pcost_t lookup(const Basic<H, W> &p) const {
            return lookup(p.get_tiles());
        }

    };

}

#endif
