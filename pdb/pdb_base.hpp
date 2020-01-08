#ifndef __PDB_BASE_HPP__
#define __PDB_BASE_HPP__

#include <array>
#include <cstdint>

#include "../sbpuzzle.hpp"

// A simple base class for different pattern database classes,
// to allow their combination via runtime polymorphism

namespace sbpuzzle {
    
    template <psize_t H, psize_t W>
    class PDB {
    public:
        virtual ~PDB() {}

        virtual uint8_t lookup(const std::array<uint8_t, H*W> &tiles) const = 0;

        uint8_t lookup(const SBPuzzle<H, W> &p) const {
            return lookup(p.get_tiles());
        }

    };

}

#endif
