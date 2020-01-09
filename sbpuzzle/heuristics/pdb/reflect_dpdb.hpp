#ifndef __REFLECTDPDB_HPP__
#define __REFLECTDPDB_HPP__

#include "pdb_base.hpp"
#include "dpdb.hpp"

namespace denizmsayin::sblock::sbpuzzle::heuristics::pdb {
    
    // A class that creates a second disjoint pattern database
    // over the provided one using a reflection through the diagonal.
    // It uses the provided database's table for looking up and 
    // does not consume any extra space, but is entirely dependent
    // on the provided database.
    //
    // Two exposed classes can be used: 
    //  - ReflectDPDBDependent: builds on top of an existing DPDB by storing a const reference
    //  - ReflectDPDBIndependent: either copies or moves an existing database and stores it
    
    namespace details {
        template <psize_t H, psize_t W, typename Storage>
        class ReflectDPDB : public PDBBase<H, W> {
        public:
            pcost_t lookup(const std::array<pcell_t, H*W> &tiles) const;

        protected:
            ReflectDPDB(const DPDB<H, W> &o_db) : db(o_db) {}
            ReflectDPDB(DPDB<H, W> &&o_db) : db(o_db) {}

        private:
            Storage db;
        };

        template <psize_t H, psize_t W>
        void tiles_reflect(const std::array<pcell_t, H*W> &tiles,
                           std::array<pcell_t, H*W> &o_tiles) 
        {
            for(psize_t i = 0, size = tiles.size(); i < size; ++i) {
                psize_t itile = tiles[i] / W, jtile = tiles[i] % W;
                psize_t rtile = jtile * W + itile;
                psize_t ipos = i / W, jpos = i % W;
                psize_t rpos = jpos * W + ipos;
                o_tiles[rpos] = rtile;
            }
        }

        template<psize_t H, psize_t W, typename Storage>
        pcost_t ReflectDPDB<H, W, Storage>::lookup(const std::array<pcell_t, H*W> &tiles) const {
            std::array<pcell_t, H*W> rtiles;
            details::tiles_reflect<H, W>(tiles, rtiles);
            return db.lookup(rtiles);
        }

    }

    template <psize_t H, psize_t W>
    class ReflectDPDBDependent : public details::ReflectDPDB<H, W, const DPDB<H, W> &> {
        typedef details::ReflectDPDB<H, W, const DPDB<H, W> &> Base;

    public:
        ReflectDPDBDependent(const DPDB<H, W> &o_db) : Base(o_db) {}
    };

    template <psize_t H, psize_t W>
    class ReflectDPDBIndependent : public details::ReflectDPDB<H, W, DPDB<H, W>> {
        typedef details::ReflectDPDB<H, W, DPDB<H, W>> Base;

    public:
        ReflectDPDBIndependent(const DPDB<H, W> &o_db) : Base(o_db) {}
        ReflectDPDBIndependent(DPDB<H, W> &&o_db) : Base(std::move(o_db)) {}
    };

}


#endif
