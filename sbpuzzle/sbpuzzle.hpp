#ifndef __SBPUZZLE_HPP__
#define __SBPUZZLE_HPP__

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

#include <cstdint>
#include <array>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <memory>
#include <limits>
#include <stack>

// TODO: consider alternatives to returning a vector for possible_actions()
// a custom iterator-like type would work nicely if it can be made light-weight

#include "sblock_utils.hpp"

namespace sbpuzzle {
    
    // publicly accessible value for _X
    const auto DONT_CARE = details::_X;

    // Zobrist hashing is great for hashing quickly. However, an important issue
    // is that hashing is not necessary for tree searching. Thus, adding hashing
    // to the puzzle when applying tree search would be a slowdown. Instead, I
    // opt to create another base class that uses Zobrist hashing while keeping
    // the original class with on-demand hashing as-is.

    template <psize_t H, psize_t W>
    class SBPuzzleBaseHashed : public SBPuzzleBase<H, W> {
    private: using Base = SBPuzzleBase<H, W>;
    protected:
        void init_hash() {
            hash_val = details::tiles_zobrist_hash<H, W>(Base::tiles);
        }

        // call after the swap between pos1 & pos2 is done
        void postupdate_hash(pcell_t pos1, pcell_t pos2) {
            // undo previous bitstrings
            hash_val ^= details::get_zobrist<H, W>(pos1, Base::tiles[pos2]);
            hash_val ^= details::get_zobrist<H, W>(pos2, Base::tiles[pos1]);
            // update with new bitstrings
            hash_val ^= details::get_zobrist<H, W>(pos1, Base::tiles[pos1]);
            hash_val ^= details::get_zobrist<H, W>(pos2, Base::tiles[pos2]);
        }

        size_t hash_val;
    };
    
    /*                                                           */

    // The derived class for the case where the hole is masked
    template <psize_t H, psize_t W>
    class SBPuzzle : public SBPuzzleBase<H, W> {
    public:

        SBPuzzle() : Base() {}

        explicit SBPuzzle(const array<pcell_t, H*W> &i_tiles) : SBPuzzle(i_tiles.begin()) {}

        // there should be H*W elements available for read starting from
        // begin, the constructor does not concern itself with type-checking
        template <typename InputItr>
        SBPuzzle(InputItr begin) {
            for(size_t i = 0; i < Base::SIZE; ++i) {
                if(*begin == Base::HOLE)
                    Base::hole_pos = i;
                Base::tiles[i] = *begin++;
            }
        }

        explicit SBPuzzle(const array<pcell_t, H*W> &i_tiles, 
                const array<bool, H*W> &mask) 
        {
            for(size_t i = 0; i < Base::SIZE; ++i) {
                if(mask[i_tiles[i]]) {
                    if(i_tiles[i] == Base::HOLE)
                        Base::hole_pos = i;
                    Base::tiles[i] = i_tiles[i];
                } else if(i_tiles[i] == Base::HOLE) {
                    throw std::invalid_argument("Constructing SBPuzzle with unmasked hole will give invalid results");
                } else {
                    Base::tiles[i] = details::_X;
                }
            }
        }

        SBPuzzle(const SBPuzzle &other) = default;
        SBPuzzle &operator=(const SBPuzzle &other) = default;

        static SBPuzzle from_binary_stream(std::istream &stream) {
            // TODO: could use istream iterators? Also update for different pcell_t
            array<uint8_t, H*W> tiles;
            stream.read(reinterpret_cast<char *>(tiles.data()), tiles.size());
            return SBPuzzle(tiles);
        }

        static SBPuzzle no_mask_goal_state() {
            array<pcell_t, H*W> tiles;
            std::iota(tiles.begin(), tiles.end(), 0);
            return SBPuzzle(tiles);
        }

        SBPuzzle goal_state() const { 
            return details::goal_state<SBPuzzle<H, W>, H, W>(Base::tiles);
        }

        // TODO: think about public/private for the nested classes
        template <class Action>
        class Generator {
        public:
        
            Generator(psize_t hp) : rec(&details::DIRECTION_RECORDS<H, W>[hp]), 
                index(hp), i(0) {}

            Generator(const Generator &) = delete;
            Generator(Generator &&) = default;

            Generator &operator=(const Generator &) = delete;
            Generator &operator=(Generator &&) = default;

            // standard Generator trio
            bool has_more() const { return i < rec->size; }
            void advance() { ++i; }
            Action value() const {
                constexpr static details::WTag<W> TAG; 
                return Action(TAG, index, rec->dirs[i]);
            }

            details::GeneratorIterator<Generator, Action> begin() { 
                return details::GeneratorIterator<Generator, Action>(this); 
            }

            details::GeneratorIterator<Generator, Action> end() const { 
                return details::GeneratorIterator<Generator, Action>(nullptr); 
            }

        private:
            // the only necessary state is the hole position
            const details::Record *rec;
            psize_t index;
            uint8_t i;
        };

        template <class Action>
        Generator<Action> action_generator() const {
            return Generator<Action>(Base::hole_pos);
        }

    private:
        using Base = SBPuzzleBase<H, W>;
    };

    // The derived class for the case where the hole is not masked
    template <psize_t H, psize_t W>
    class SBPuzzleNoHole : public SBPuzzleBase<H, W> {
    public:

        // constructs with only hole unmasked, equivalent to SBPuzzle in that case
        SBPuzzleNoHole() : Base() {}

        explicit SBPuzzleNoHole(const array<pcell_t, H*W> &i_tiles, 
                                const array<bool, H*W> &mask) 
        {
            for(size_t i = 0; i < Base::SIZE; ++i) {
                if(i_tiles[i] == Base::HOLE) {
                    Base::tiles[i] = Base::HOLE;
                    Base::hole_pos = i;
                } else if(mask[i_tiles[i]]) {
                    if(i_tiles[i] == Base::HOLE) {
                        throw std::invalid_argument("Constructing SBPuzzleNoHole with "
                                                    "a masked hole will give invalid results");
                    }
                    Base::tiles[i] = i_tiles[i];
                } else {
                    Base::tiles[i] = details::_X;
                }
            }
            prop_hole();
        }

        SBPuzzleNoHole(const SBPuzzleNoHole &other) = default;
        SBPuzzleNoHole &operator=(const SBPuzzleNoHole &other) = default;

        SBPuzzleNoHole goal_state() const {
            return details::goal_state<SBPuzzleNoHole<H, W>, H, W>(Base::tiles);
        }

        pcost_t apply_action(TileSwapAction a) {
            pcost_t cost = Base::apply_action(a);
            reprop_hole(); // repropagate the hole
            return cost; // cost is always unit
        }

        /*
        template <class Action>
        typename Action::iterator actions_begin() const {
            return PADelegate<Action>::b(*this);
        }


        template <class Action>
        typename Action::iterator actions_end() const {
            return PADelegate<Action>::e(*this);
        }
        */

        template <typename Action, typename Dummy = void>
        class Generator;

        template <typename Action>
        Generator<Action> action_generator() const {
            return Generator<Action>(*this);
        }

    private:
        using Base = SBPuzzleBase<H, W>;

        
        // TODO: consider caching hole/tile positions for faster searching

        void prop_hole() {
            details::tiles_prop_hole<H, W>(Base::tiles, Base::hole_pos);
        }

        void reprop_hole() { // re-do hole propagation after a hole position update
            details::tiles_reprop_hole<H, W>(Base::tiles, Base::hole_pos);
        }

        template <typename Action, typename Dummy = void>
        struct PADelegate {
            static typename Action::iterator b(const SBPuzzleNoHole &p);
            static typename Action::iterator e(const SBPuzzleNoHole &p);
        };
    };


    template <psize_t H, psize_t W>
    template <typename Dummy>
    class SBPuzzleNoHole<H, W>::Generator<TileSwapAction, Dummy> {
    public:
        using GeneratorIterator = details::GeneratorIterator<Generator, TileSwapAction>;

        Generator(const SBPuzzleNoHole<H, W> &puzzle) 
            : tiles(puzzle.get_tiles().data()), index(0), found_pos(0), i(-1)
        {
            advance_until_possible_index();
            advance();
        }

        Generator(const Generator &) = delete;
        Generator(Generator &&) = default;

        Generator &operator=(const Generator &) = delete;
        Generator &operator=(Generator &&) = default;

        bool has_more() const {
            return index < details::SIZE<H, W>;
        }

        void advance() {
            using namespace details;
            // invariant: currently on cell where a move has been previously found at i
            ++i; // advance the index on the current cell
            while(index < SIZE<H, W>) {
                const Record &rec = DIRECTION_RECORDS<H, W>[index];
                // try finding a move on the current cell
                while(i < rec.size) {
                    // if there are still cells we can check around this one
                    Direction dir = rec.dirs[i];
                    psize_t check_pos = index + OFFSETS<W>[static_cast<uint8_t>(dir)];
                    if(tiles[check_pos] == HOLE<H, W>) { // can swap tiles if it is a hole
                        found_pos = check_pos;
                        return;
                    }
                    else {
                        ++i;
                    }
                }
                // current cell exhausted, get the next cell 
                ++index; // move one step further
                advance_until_possible_index();
                i = 0;
            }

        }

        TileSwapAction value() const {
            return TileSwapAction(index, found_pos);
        }

        GeneratorIterator begin() {
            return GeneratorIterator(this);
        }

        GeneratorIterator end() const {
            return GeneratorIterator(nullptr);
        }

    private:
        void advance_until_possible_index() { // move on to the next non-hole tile
            while(index < details::SIZE<H, W>  // keep moving until a viable tile
                  && tiles[index] == details::HOLE<H, W>) 
                ++index;
        }

        const pcell_t *tiles;
        psize_t index;
        psize_t found_pos;
        uint8_t i;

    };



    /*
    // PADelegate specialization for the TileSwapAction class
    template <psize_t H, psize_t W>
    template <typename Dummy>
    struct SBPuzzleNoHole<H, W>::PADelegate<TileSwapAction, Dummy> {

        static void f(const SBPuzzleNoHole<H, W> &p) {
            // check around propagated hole tiles
            for(size_t i = 0; i < p.SIZE; ++i) {
                if(p.tiles[i] == p.HOLE) { // only actual tiles
                    array<bool, 4> valid;
                    details::tiles_mark_valid_moves<H, W>(i, valid);
                    for(size_t dir = 0; dir < 4; ++dir) { // check the 4-neighborhood
                        if(valid[dir]) {
                            auto np = i + details::OFFSETS<W>[dir]; // candidate swap position
                            if(p.tiles[np] != p.HOLE) // can swap if the neighbor is a tile
                                *out++ = TileSwapAction(np, i);
                            // No need to check for _X, since holes propagate over those
                        }
                    }
                }
            }
        }
    };
    */


    /*
    // TODO: remove this after the switch to tagged union is done
    template <psize_t H, psize_t W>
    using SBPuzzle = SBPuzzle<H, W>;
    */
} 

// add hashability
namespace std {
    template <sbpuzzle::psize_t H, sbpuzzle::psize_t W>
    struct hash<sbpuzzle::SBPuzzle<H, W>> 
        : sbpuzzle::details::hash<sbpuzzle::SBPuzzle<H, W>> {};

    template <sbpuzzle::psize_t H, sbpuzzle::psize_t W>
    struct hash<sbpuzzle::SBPuzzleNoHole<H, W>> 
        : sbpuzzle::details::hash<sbpuzzle::SBPuzzleNoHole<H, W>> {};
    
}

#endif
