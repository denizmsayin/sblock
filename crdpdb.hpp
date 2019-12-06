#ifndef __CRDPDB_HPP__
#define __CRDPDB_HPP__


#include "pdb.hpp"
#include "dpdb.hpp"
#include "reflectdpdb.hpp"
#include "combineddb.hpp"

// a convenience class that provides the same constructors as the DPDB
// class and creates a reflected DB from it and combines the two, 
// inside a single constructor

namespace sbpuzzle {

    template <psize_t H, psize_t W>
    class CRDPDB : public PDB<H, W> {
    public:

        static CRDPDB from_file(const std::string &filename) {
            return CRDPDB(DPDB<H, W>::from_file(filename));
        }

        static CRDPDB generate(const std::array<uint8_t, H*W> &o_groups) {
            return CRDPDB(DPDB<H, W>::generate(o_groups));
        }

        uint8_t lookup(const std::array<uint8_t, H*W> &tiles) const {
            return cdb.lookup(tiles);
        }

    private:

        // using an rvalue reference because deep-copying 
        // a whole db would be expensive & unnecessary
        CRDPDB(DPDB<H, W> &&o_dpdb)
            : dpdb(o_dpdb), rdb(dpdb), cdb(std::vector<const PDB<H, W> *> {&dpdb, &rdb}) {}

        DPDB<H, W> dpdb;
        ReflectDPDB<H, W> rdb;
        CombinedDB<H, W> cdb;
    };
    
}



#endif 
