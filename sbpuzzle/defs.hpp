#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_DEFS_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_DEFS_HPP

#include <iostream>
#include <array>

namespace denizmsayin::sblock::sbpuzzle {
        
    typedef int8_t spsize_t;
    typedef uint8_t psize_t;
    typedef uint8_t pcell_t;
    typedef uint8_t pcost_t;

    namespace details {
        // This namespace contains implementation details, with both
        // basic utility functions as well as C-like functions
        // that work directly on the tile array, to be called by
        // the puzzle classes
        
        constexpr pcell_t _X = std::numeric_limits<pcell_t>::max();

        template <psize_t W>
        static constexpr spsize_t OFFSETS[] = {-W, +1, +W, -1};
        // offset applied to moves allowed by tiles_mark_valid_moves

        template <psize_t H, psize_t W>
        static constexpr psize_t SIZE = H*W;

        template <psize_t H, psize_t W>
        static constexpr psize_t HOLE = H*W-1;
        
        enum class Direction : uint8_t {
            UP, RIGHT, DOWN, LEFT
        };

        std::ostream &operator<<(std::ostream &os, Direction dir) {
            switch(dir) {
                case Direction::UP:     os << "U"; break;
                case Direction::RIGHT:  os << "R"; break;
                case Direction::DOWN:   os << "D"; break;
                case Direction::LEFT:   os << "L"; break;
                default: throw std::runtime_error("Unexpected direction in operator<<"
                                                  "(std::ostream &, Direction)");
            }
            return os;
        }

        std::array<Direction, 4> DIRECTION_REVERSE {
            Direction::DOWN,
            Direction::LEFT,
            Direction::UP,
            Direction::RIGHT
        };

    }

    // publicly accessible value for _X
    const auto DONT_CARE = details::_X;
}

#endif
