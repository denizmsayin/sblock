#ifndef __COMBINEDDB_HPP__
#define __COMBINEDDB_HPP__

#include <array>
#include <vector>

#include "pdb_base.hpp"

namespace denizmsayin::sblock::sbpuzzle::heuristics::pdb {

    template <psize_t H, psize_t W>
    class CombinedDB : public PDBBase<H, W> {
    public:
        CombinedDB(const std::vector<const PDBBase<H, W> *> &o_dbs) : dbs(o_dbs) {}

        pcost_t lookup(const std::array<pcell_t, H*W> &tiles) const;

    private:
        std::vector<const PDBBase<H, W> *> dbs;
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
