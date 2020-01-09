#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_MASKEDWHOLE_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_MASKEDWHOLE_HPP

// A class that defines the sliding block puzzle in a manner solvable by
// our pre-defined search functions. Templates are used for the height
// and width of the puzzle to extract large performance gains from
// extra optimization.

// Let us consider three possible 'subtypes' of our puzzle:
// - The standard puzzle
// - The puzzle with some elements 'masked out' as X
// - The puzzle with some elements & the hole masked out
// All three are slightly different, but the first one is actually
// a subset of the second one. However, the hole being masked out
// changes the set of possible actions quite significantly. The ideal
// solution for dealing with this would be having most of the content
// in a base class and deriving the extra functionality via virtual
// functions. This however renders our search function unusable due
// to them not working with pointers. 
//
// The issue with this design is that it does not make much sense
// to have inheritance in the puzzle types, they are actually the same
// puzzle. The only difference lies in how we generate and apply moves
// for those puzzles. Even the application of moves could be deemed the
// same depending on the 'move' class. However, the algorithm for
// generating the moves remains different.
//
// I would also like it to be possible for different types of moves
// being possibly defined for each class. However, since this is 
// static typed language, the only possibility seems to also have
// actions as members of an 'interface' and have derived action classes.
// All that run-time dynamic allocation and polymorphism will slow
// the program down significantly.
//
// The best approach for this specific case seems to be acting restrictively so
// that all dynamic stuff can be resolved at compile time. A different static
// type is generated using a branch before calling the constructor. Also,
// the possible_actions() function of the class remains extensible via templates
// as before, entirely statically.
//

#include "basic.hpp"

namespace denizmsayin::sblock::sbpuzzle {
    // The derived class for the case where the hole is masked
    template <psize_t H, psize_t W>
    class MaskedWithHole : public Basic<H, W> {
    private:
        using B = Basic<H, W>;
        
        // uninitialized: careful!
        MaskedWithHole() : B() {}

    public:

        // since tiles.hpp functions already work with masked puzzles 
        // (containing DONT_CARE values), this class only has few differences
        // from Basic<H, W>

        // inherited as-is:              OPINION
        // get_tiles()                      +
        // operator==                       +
        // hash()                           +
        // apply_action(TileSwapAction)     +
        // apply_action(DirectionAction)    +
        // apply_action_sequence(...)       +
        // operator<<                       +
        // to_binary_stream                 +
        // from_binary_stream               +
        // template action_generator()      +


        static MaskedWithHole uninitialized() { return MaskedWithHole(); }

        MaskedWithHole(const MaskedWithHole &other) = default;
        MaskedWithHole(MaskedWithHole &&other) = default;
        MaskedWithHole &operator=(const MaskedWithHole &other) = default;
        
        template <typename CellInputIterator, typename BoolRandomIterator>
        MaskedWithHole(CellInputIterator cbegin, BoolRandomIterator mask) : B(cbegin)
        {
            if(!mask[B::HOLE])
                throw std::invalid_argument("Constructing MaskedWithHole, but hole is not in"
                                            " the provided mask.");
            for(size_t i = 0; i < B::SIZE; ++i) 
                if(!mask[B::tiles[i]])
                    B::tiles[i] = DONT_CARE;
        }

        explicit MaskedWithHole(const std::array<pcell_t, H*W> &i_tiles, 
                                const std::array<bool, H*W> &mask) 
            : MaskedWithHole(i_tiles.begin(), mask.begin()) {}

        MaskedWithHole goal_state() const { 
            return details::goal_state<MaskedWithHole<H, W>, H, W>(B::tiles);
        }

    };
}

namespace std {
    template <denizmsayin::sblock::sbpuzzle::psize_t H, denizmsayin::sblock::sbpuzzle::psize_t W>
    struct hash<denizmsayin::sblock::sbpuzzle::MaskedWithHole<H, W>> 
        : denizmsayin::sblock::sbpuzzle::details::hash<denizmsayin::sblock::sbpuzzle::MaskedWithHole<H, W>> {};
};

#endif
