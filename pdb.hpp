#ifndef __PDB_HPP__
#define __PDB_HPP__

#include <array>
#include <cstdint>

// A simple base class for different pattern database classes,
// to allow their combination via runtime polymorphism

// TODO: refactor psize_t

namespace sbpuzzle {
    template <uint8_t H, uint8_t W>
    class PDB {
    public:
        virtual ~PDB() {}

        virtual uint8_t lookup(const std::array<uint8_t, H*W> &tiles) const = 0;
    };
}

#endif
