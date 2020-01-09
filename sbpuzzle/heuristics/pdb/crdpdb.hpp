#ifndef __CRDPDB_HPP__
#define __CRDPDB_HPP__


#include "pdb_base.hpp"
#include "dpdb.hpp"
#include "reflect_dpdb.hpp"
#include "combined_pdb.hpp"

// a convenience class that provides the same constructors as the DPDB
// class and creates a reflected DB from it and combines the two, 
// inside a single constructor

namespace denizmsayin::sblock::sbpuzzle::heuristics::pdb {

    template <psize_t H, psize_t W>
    class CRDPDB : public PDBBase<H, W> {
    public:

        // using an rvalue reference because deep-copying 
        // a whole db would be expensive & unnecessary
        CRDPDB(DPDB<H, W> &&o_dpdb)
            : dpdb(o_dpdb), rdb(dpdb), cdb(std::vector<const PDBBase<H, W> *> {&dpdb, &rdb}) {}

        static CRDPDB from_file(const std::string &filename) {
            return CRDPDB(DPDB<H, W>::from_file(filename));
        }

        static CRDPDB generate(const std::array<pcell_t, H*W> &o_groups) {
            return CRDPDB(DPDB<H, W>::generate(o_groups));
        }

        pcost_t lookup(const std::array<pcell_t, H*W> &tiles) const {
            return cdb.lookup(tiles);
        }

    private:

        DPDB<H, W> dpdb;
        ReflectDPDBDependent<H, W> rdb;
        CombinedDB<H, W> cdb;
    };
    
}



#endif 
