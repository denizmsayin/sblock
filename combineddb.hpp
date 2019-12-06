#ifndef __COMBINEDDB_HPP__
#define __COMBINEDDB_HPP__

#include <array>
#include <vector>

#include "pdb.hpp"

namespace sbpuzzle {

    template <psize_t H, psize_t W>
    class CombinedDB : public PDB<H, W> {
    public:
        CombinedDB(const std::vector<const PDB<H, W> *> &o_dbs) : dbs(o_dbs) {}

        uint8_t lookup(const std::array<uint8_t, H*W> &tiles) const;

    private:
        std::vector<const PDB<H, W> *> dbs;
    };
    
    template <psize_t H, psize_t W>
    uint8_t CombinedDB<H, W>::lookup(const std::array<uint8_t, H*W> &tiles) const {
        uint8_t max_val = 0;
        for(const auto *db : dbs) {
            uint8_t lval = db->lookup(tiles);
            max_val = (lval > max_val) ? lval : max_val;
        }
        return max_val;
    }

}

#endif 
