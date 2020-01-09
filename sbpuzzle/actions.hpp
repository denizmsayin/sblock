#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_ACTIONS_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_ACTIONS_HPP

#include "defs.hpp"

namespace denizmsayin::sblock::sbpuzzle {
        
    namespace details {

        // used for tag-dispatching on the TileSwapAction constructor
        template <psize_t W>
        struct WTag {
            psize_t w = W;
        };

    }

    struct TileSwapAction {
        psize_t tpos, hpos;

        TileSwapAction() = default;
        TileSwapAction(psize_t t, psize_t h) : tpos(t), hpos(h) {}

        // Since constructors have no names, we cannot have templated constructors
        // with no method of argument deduction. Thus, I need a dummy type to pass
        // to the constructor for deducing the template argument W.
        template <psize_t W>
        TileSwapAction(details::WTag<W> tag, psize_t index, details::Direction dir) :
            tpos(index + details::OFFSETS<W>[static_cast<size_t>(dir)]), hpos(index) {}
            
        static TileSwapAction reverse(TileSwapAction a) { 
            return TileSwapAction(a.hpos, a.tpos); 
        }

        bool operator==(TileSwapAction a) const { return a.hpos == hpos && a.tpos == tpos; }
        bool operator!=(TileSwapAction a) const { return a.hpos != hpos || a.tpos != tpos; }
    };

    std::ostream &operator<<(std::ostream &os, TileSwapAction a) {
        os << "(t:" << static_cast<unsigned>(a.hpos) 
           << ", h:" << static_cast<unsigned>(a.tpos) << ")";
        return os;
    }

    struct DirectionAction {
        using Direction = details::Direction;

        Direction dir;

        DirectionAction() = default;
        DirectionAction(Direction d) : dir(d) {}

        template <psize_t W>
        DirectionAction(details::WTag<W> tag, psize_t hole_pos, Direction d) : DirectionAction(d) {}

        static DirectionAction reverse(DirectionAction a) {
            return details::DIRECTION_REVERSE[static_cast<uint8_t>(a.dir)];
        }

        bool operator==(DirectionAction d) const { return dir == d.dir; }
        bool operator!=(DirectionAction d) const { return dir != d.dir; }
    };

    std::ostream &operator<<(std::ostream &os, DirectionAction a) {
        os << a.dir;
        return os;
    }

}

#endif
