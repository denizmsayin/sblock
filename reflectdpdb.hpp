#ifndef __REFLECTDPDB_HPP__
#define __REFLECTDPDB_HPP__

#include "pdb.hpp"
#include "dpdb.hpp"

namespace sbpuzzle {
    
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
        class ReflectDPDB : public PDB<H, W> {
        public:
            uint8_t lookup(const std::array<uint8_t, H*W> &tiles) const;

        protected:
            ReflectDPDB(const DPDB<H, W> &o_db) : db(o_db) {}
            ReflectDPDB(DPDB<H, W> &&o_db) : db(o_db) {}

        private:
            Storage db;
        };

        template <psize_t H, psize_t W>
        void tiles_reflect(const std::array<uint8_t, H*W> &tiles,
                           std::array<uint8_t, H*W> &o_tiles) 
        {
            for(size_t i = 0, size = tiles.size(); i < size; ++i) {
                uint8_t itile = tiles[i] / W, jtile = tiles[i] % W;
                uint8_t rtile = jtile * W + itile;
                size_t ipos = i / W, jpos = i % W;
                size_t rpos = jpos * W + ipos;
                o_tiles[rpos] = rtile;
            }
        }

        template<psize_t H, psize_t W, typename Storage>
        uint8_t ReflectDPDB<H, W, Storage>::lookup(const std::array<uint8_t, H*W> &tiles) const {
            std::array<uint8_t, H*W> rtiles;
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
