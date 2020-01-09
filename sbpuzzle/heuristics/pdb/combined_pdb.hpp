#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_HEURISTICS_PDB_COMBINED_PDB_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_HEURISTICS_PDB_COMBINED_PDB_HPP

#include <array>
#include <vector>

#include "base.hpp"

namespace denizmsayin::sblock::sbpuzzle::heuristics::pdb {

    template <psize_t H, psize_t W>
    class CombinedDB : public Base<H, W> {
    public:
        CombinedDB(const std::vector<const Base<H, W> *> &o_dbs) : dbs(o_dbs) {}

        pcost_t lookup(const std::array<pcell_t, H*W> &tiles) const;

    private:
        std::vector<const Base<H, W> *> dbs;
    };
    
    template <psize_t H, psize_t W>
    pcost_t CombinedDB<H, W>::lookup(const std::array<pcell_t, H*W> &tiles) const {
        pcost_t max_val = 0;
        for(const auto *db : dbs) {
            pcost_t lval = db->lookup(tiles);
            max_val = (lval > max_val) ? lval : max_val;
        }
        return max_val;
    }

}

#endif 
