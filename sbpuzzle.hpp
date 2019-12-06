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

// TODO: consider alternatives to returning a vector for possible_actions()
// a custom iterator-like type would work nicely if it can be made light-weight

#include "pdb.hpp"

namespace sbpuzzle {
        
    using std::array;

    struct TileSwapAction {
        uint8_t tpos, hpos;

        TileSwapAction(uint8_t t, uint8_t h) : tpos(t), hpos(h) {}

        TileSwapAction inverse(TileSwapAction a) {
            return TileSwapAction(a.hpos, a.tpos);
        }
    };

    typedef uint8_t psize_t;
    
    namespace details {
        // This namespace contains implementation details, with both
        // basic utility functions as well as C-like functions
        // that work directly on the tile array, to be called by
        // the puzzle classes
        
        constexpr uint8_t _X = 255;

        template <psize_t W>
        static constexpr int8_t OFFSETS[] = {-W, +1, +W, -1};
        // offset applied to moves allowed by tiles_mark_valid_moves

        template <psize_t H, psize_t W>
        static constexpr uint8_t SIZE = H*W;

        template <psize_t H, psize_t W>
        static constexpr uint8_t HOLE = H*W-1;

        template <psize_t H, psize_t W>
        inline void tiles_correct_fill(array<uint8_t, H*W> &tiles) {
            for(size_t size = tiles.size(), i = 0; i < size; ++i)
                tiles[i] = i;
        }

        template <psize_t H, psize_t W>
        inline void tiles_correct_fill(array<uint8_t, H*W> &tiles, 
                                       const array<bool, H*W> &mask) 
        {
            for(size_t size = tiles.size(), i = 0; i < size; ++i)
                tiles[i] = mask[i] ? i : _X;
        }

        template <psize_t H, psize_t W>
        inline void tiles_reconstruct_mask(const array<uint8_t, H*W> &tiles, 
                                           array<bool, H*W> &mask) 
        {
            std::fill(mask.begin(), mask.end(), false);
            for(size_t size = tiles.size(), i = 0; i < size; ++i)
                if(tiles[i] != _X)
                    mask[tiles[i]] = true;
        }

        template <psize_t H, psize_t W>
        inline bool tiles_in_correct_places(const array<uint8_t, H*W> &tiles) {
            for(size_t size = tiles.size(), i = 0; i < size; ++i) 
                if(tiles[i] != _X && tiles[i] != HOLE<H, W> && tiles[i] != i)
                    return false;
            return true;
        }

        template <psize_t H, psize_t W>
        inline bool tiles_equal(const array<uint8_t, H*W> &t1, const array<uint8_t, H*W> &t2) {
            return std::equal(t1.begin(), t1.end(), t2.begin());
        }

        template <psize_t H, psize_t W>
        size_t tiles_hash(const array<uint8_t, H*W> tiles) {
            return hash_byte_array(&tiles[0], H*W);
        }

        template <psize_t H, psize_t W>
            std::ostream& tiles_stream(
                    std::ostream &s, 
                    const array<uint8_t, H*W> &tiles) 
            {
                // custom comparator that does not care about the _X value, takes it as min
                auto cmp = [](uint8_t a, uint8_t b) -> bool {
                    // _X (don't care) values are replaced by 0
                    a = (a == _X) ? 0 : a;
                    b = (b == _X) ? 0 : b;
                    return a < b;
                };
                const uint8_t *max = std::max_element(tiles.begin(), tiles.end(), cmp);
                uint8_t num_digits = (*max > 99) ? 3 : ((*max > 9) ? 2 : 1); 
                int num_dashes = W * (num_digits + 3) + 1;
                std::string dash_str(num_dashes, '-');
                std::string empty_str(num_digits - 1, ' ');
                for(size_t i = 0, k = 0; i < H; i++) {
                    s << dash_str << std::endl;
                    for(size_t j = 0; j < W; j++) {
                        s << "| "; 
                        if(tiles[k] == _X)
                            s << empty_str << "X ";
                        else 
                            s << std::setw(num_digits) << static_cast<int>(tiles[k]) << " ";
                        ++k;
                    }
                    s << "|" << std::endl;
                }
                s << dash_str;
                return s;
            }

        template <psize_t H, psize_t W>
            uint64_t tiles_manhattan_distance_to_solution(const array<uint8_t, H*W> &tiles) {
                uint64_t dist = 0;
                for(size_t size = tiles.size(), i = 0; i < size; ++i) {
                    if(tiles[i] != _X && tiles[i] != HOLE<H, W>) {
                        // open to optimisation, but no need
                        uint8_t row = i / W, col = i % W;
                        uint8_t actual_row = tiles[i] / W, actual_col = tiles[i] % W;
                        dist += abs(row - actual_row) + abs(col - actual_col);
                    }
                }
                return dist;
            }

        template <psize_t H, psize_t W>
            void tiles_mark_valid_moves(uint8_t p, array<bool, 4> &directions) {
                int rem = p % W;
                directions[0] = p >= W; // up
                directions[1] = rem < W-1; // right
                directions[2] = p < SIZE<H, W> - W; // down
                directions[3] = rem > 0; // left
            }

        template <psize_t H, psize_t W>
            size_t count_inversions(const array<uint8_t, H*W> &tiles) {
                size_t inv = 0;
                for(size_t i = 0, size = tiles.size(); i < size; ++i) 
                    if(tiles[i] != HOLE<H, W>) 
                        for(size_t j = i+1; j < size; ++j) 
                            if(tiles[j] != HOLE<H, W> && tiles[i] > tiles[j]) 
                                ++inv;
                return inv;
            }

        // simple struct to inherit from while extending std::hash
        // for our defined types, since they all have a hash() func
        template <class P>
        struct hash {
            size_t operator()(P const &p) const noexcept { 
                return p.hash(); 
            }
        };


        // template polymorphic goal_state base function
        template <class P, psize_t H, psize_t W>
            P goal_state(const array<uint8_t, H*W> &tiles) {
                // this function should not be called often, so it seems
                // to me it makes more sense to reconstruct the mask array
                // here rather than store it in each puzzle instance
                array<bool, SIZE<H, W>> mask;
                tiles_reconstruct_mask<H, W>(tiles, mask);
                array<uint8_t, SIZE<H, W>> tiles2;
                tiles_correct_fill<H, W>(tiles2, mask);
                return P(tiles2, mask);
            }

        // hole propagation & variants for the no-hole puzzle version
        template <psize_t H, psize_t W>
            void tiles_prop_hole(array<uint8_t, H*W> &tiles, uint8_t i) {
                array<bool, 4> valid;
                tiles_mark_valid_moves<H, W>(i, valid);
                for(auto dir = 0; dir < 4; ++dir) {
                    if(valid[dir]) {
                        auto np = i + OFFSETS<W>[dir];
                        if(tiles[np] == _X) {
                            tiles[np] = HOLE<H, W>;
                            tiles_prop_hole<H, W>(tiles, np);
                        }
                    }
                }
            }

        // re-do hole propagation after a hole position update
        template <psize_t H, psize_t W>
            void tiles_reprop_hole(array<uint8_t, H*W> &tiles, uint8_t new_hole_pos) {
                for(size_t i = 0, size = tiles.size(); i < size; ++i)
                    if(tiles[i] == HOLE<H, W>)
                        tiles[i] = _X;
                tiles[new_hole_pos] = HOLE<H, W>;
                tiles_prop_hole<H, W>(tiles, new_hole_pos);
            }

        // collapse hole for extended index calculation by DPDB
        template <psize_t H, psize_t W>
            void tiles_collapse_hole(array<uint8_t, H*W> &tiles) {
                bool hole_found = false;
                // only leave the hole at the first position
                for(size_t i = 0, size = tiles.size(); i < size; ++i) {
                    if(tiles[i] == HOLE<H, W>) {
                        if(hole_found)
                            tiles[i] = _X;
                        else
                            hole_found = true;
                    }
                }
            }

        template <psize_t H, psize_t W, class OutputIterator>
            void tiles_fill_group(const std::array<bool, H*W> &group_mask, 
                    const std::array<uint8_t, H*W> &tiles, 
                    OutputIterator itr)
            {
                for(size_t i = 0, size = tiles.size(); i < size; ++i)
                    if(tiles[i] != _X && group_mask[tiles[i]])
                        *itr++ = tiles[i];
            }




    }

    // publicly accessible value for _X
    const auto DONT_CARE = details::_X;

    // a utility function to decide if a set of tiles is solvable
    template <psize_t H, psize_t W>
        bool tiles_solvable(const array<uint8_t, H*W> &tiles) {
            // see https://www.cs.bham.ac.uk/~mdr/teaching/modules04/java2/TilesSolvability.html
            uint8_t hole_pos = std::find(tiles.begin(), tiles.end(), details::HOLE<H, W>) 
                - tiles.begin();
            size_t inversions = details::count_inversions<H, W>(tiles);
            size_t hole_row_from_bottom = H - hole_pos / W;
            bool grid_width_odd = (W & 1);
            bool inversions_even = !(inversions & 1);
            bool blank_on_odd_row = (hole_row_from_bottom & 1);
            return ((grid_width_odd && inversions_even) ||
                    ((!grid_width_odd) && (blank_on_odd_row == inversions_even)));
        }

    // I've elected to define the functions right in the declaration since
    // writing them separately takes time and I keep changing implementations...

    // Since some functions are exactly the same for cases where the hole is
    // in the group and not, we have a base class that prevents us from
    // repeating the same functions a few times. The class is only
    // instantiable by the derived classes as its constructor is protected.
    // The derived classes still have to implement constructors, possible_actions(),
    // and apply_action()
    template <psize_t H, psize_t W>
        class SBPuzzleBase {
            public:

                bool is_solved() const {
                    return details::tiles_in_correct_places<H, W>(tiles);
                }

                // Note: these two functions should be implemented by the derived classes
                // same implementation for derived classes
                // base class PADelegate should not implement anything
                // template <class Action>
                // std::vector<Action> possible_actions() const;
                // uint64_t apply_action(TileSwapAction a);

                bool operator==(const SBPuzzleBase &other) const {
                    return details::tiles_equal<H, W>(tiles, other.tiles);
                }

                size_t hash() const {
                    return details::tiles_hash<H, W>(tiles);
                }

                template <psize_t HH, psize_t WW>
                friend std::ostream &operator<<(std::ostream &s, const SBPuzzleBase<HH, WW> &p) {
                    return details::tiles_stream<HH, WW>(s, p.tiles);
                }

                std::ostream &stream_binary(std::ostream &s) const {
                    const char *out_ptr = reinterpret_cast<const char *>(&tiles[0]);
                    size_t out_size = tiles.size() * sizeof(tiles[0]);
                    return s.write(out_ptr, out_size);
                }

                static size_t tile_size_in_bytes() {
                    return sizeof(tiles);
                }

                int manhattan_distance_to_solution() const {
                    return details::tiles_manhattan_distance_to_solution<H, W>(tiles);
                }

                uint8_t lookup_cost(const PDB<H, W> *pdb) const {
                    return pdb->lookup(tiles);
                }

                template <typename PDBType>
                size_t determine_index(uint8_t group_no, const PDBType &pdb) const {
                    return pdb.calculate_table_index(group_no, tiles);
                }

                uint64_t apply_action(TileSwapAction a) {
                    tiles[a.hpos] = tiles[a.tpos]; // move tile over the hole
                    tiles[a.tpos] = HOLE; // tile is now the hole
                    hole_pos = a.tpos;
                    return 1; // cost is always unit
                }

            protected:
                // only callable by derived classes
                SBPuzzleBase() : hole_pos(details::HOLE<H, W>) {
                    details::tiles_correct_fill<H, W>(tiles);
                }

                static constexpr auto SIZE = details::SIZE<H, W>;
                static constexpr auto HOLE = details::HOLE<H, W>;

                array<uint8_t, SIZE> tiles;
                psize_t hole_pos;
        };

    /*                                                           */

    // The derived class for the case where the hole is masked
    template <psize_t H, psize_t W>
    class SBPuzzleWHole : public SBPuzzleBase<H, W> {
    public:

        SBPuzzleWHole() : Base() {}

        explicit SBPuzzleWHole(const array<uint8_t, H*W> &i_tiles) {
            for(size_t i = 0; i < Base::SIZE; ++i) {
                if(i_tiles[i] == Base::HOLE)
                    Base::hole_pos = i;
                Base::tiles[i] = i_tiles[i];
            }
        }

        explicit SBPuzzleWHole(const array<uint8_t, H*W> &i_tiles, 
                const array<bool, H*W> &mask) 
        {
            for(size_t i = 0; i < Base::SIZE; ++i) {
                if(mask[i_tiles[i]]) {
                    if(i_tiles[i] == Base::HOLE)
                        Base::hole_pos = i;
                    Base::tiles[i] = i_tiles[i];
                } else if(i_tiles[i] == Base::HOLE) {
                    throw std::invalid_argument("Constructing SBPuzzleWHole with unmasked hole will give invalid results");
                } else {
                    Base::tiles[i] = details::_X;
                }
            }
        }

        SBPuzzleWHole(const SBPuzzleWHole &other) = default;
        SBPuzzleWHole &operator=(const SBPuzzleWHole &other) = default;

        SBPuzzleWHole goal_state() const { 
            return details::goal_state<SBPuzzleWHole<H, W>, H, W>(Base::tiles);
        }

        template <class Action>
        std::vector<Action> possible_actions() const {
            return PADelegate<Action>::f(*this); // delegate to the struct
        }

    private:
        using Base = SBPuzzleBase<H, W>;

        // since it is not possible to specialize a templated function of an unspecialized
        // templated class, in our case the possible_actions() function, we need a dummy
        // templated wrapper struct to which we can delegate possible_actions(). However,
        // it is not possible to FULLY specialize the wrapper struct either, only partially.
        // Which is why we need a struct with the actual template parameter and a dummy one.
        // C++ hell, anyone?! Wow, templates are C++'s strength but also quite messed up.
        template <typename Action, typename Dummy = void>
        struct PADelegate {
            static std::vector<Action> f(const SBPuzzleWHole &p);
        };
    };
 
    // PADelegate specialization for the TileSwapAction class
    template <psize_t H, psize_t W>
    template <typename Dummy>
    struct SBPuzzleWHole<H, W>::PADelegate<TileSwapAction, Dummy> {
        static auto f(const SBPuzzleWHole<H, W> &p) {
            auto hp = p.hole_pos;
            std::vector<TileSwapAction> actions;
            std::array<bool, 4> valid;
            details::tiles_mark_valid_moves<H, W>(hp, valid);
            for(size_t dir = 0; dir < 4; ++dir)
                if(valid[dir])
                    actions.emplace_back(hp + details::OFFSETS<W>[dir], hp);
            return actions;
        }
    };

    /*                                                           */
   
    // The derived class for the case where the hole is not masked
    template <psize_t H, psize_t W>
    class SBPuzzleNoHole : public SBPuzzleBase<H, W> {
    public:

        // constructs with only hole unmasked, equivalent to SBPuzzleWHole in that case
        SBPuzzleNoHole() : Base() {}

        explicit SBPuzzleNoHole(const array<uint8_t, H*W> &i_tiles, 
                                const array<bool, H*W> &mask) 
        {
            for(size_t i = 0; i < Base::SIZE; ++i) {
                if(i_tiles[i] == Base::HOLE) {
                    Base::tiles[i] = Base::HOLE;
                    Base::hole_pos = i;
                } else if(mask[i_tiles[i]]) {
                    if(i_tiles[i] == Base::HOLE)
                        throw std::invalid_argument("Constructing SBPuzzleNoHole with a masked hole will give invalid results");
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

        template <class Action>
        std::vector<Action> possible_actions() const {
            return PADelegate<Action>::f(*this); // delegate to the struct
        }

        uint64_t apply_action(TileSwapAction a) {
            uint64_t cost = Base::apply_action(a);
            reprop_hole(); // repropagate the hole
            return cost; // cost is always unit
        }

        template <typename PDBType>
        size_t determine_extended_index(uint8_t group_no, const PDBType &pdb) const {
            return pdb.calculate_table_index(group_no, Base::tiles, true);
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
            static std::vector<Action> f(const SBPuzzleNoHole &p);
        };
    };

    // PADelegate specialization for the TileSwapAction class
    template <psize_t H, psize_t W>
    template <typename Dummy>
    struct SBPuzzleNoHole<H, W>::PADelegate<TileSwapAction, Dummy> {
        static auto f(const SBPuzzleNoHole<H, W> &p) {
            std::vector<TileSwapAction> actions;
            // check around propagated hole tiles
            for(size_t i = 0; i < p.SIZE; ++i) {
                if(p.tiles[i] == p.HOLE) { // only actual tiles
                    array<bool, 4> valid;
                    details::tiles_mark_valid_moves<H, W>(i, valid);
                    for(size_t dir = 0; dir < 4; ++dir) { // check the 4-neighborhood
                        if(valid[dir]) {
                            auto np = i + details::OFFSETS<W>[dir]; // candidate swap position
                            if(p.tiles[np] != p.HOLE) // can swap if the neighbor is a tile
                                actions.emplace_back(np, i);
                            // No need to check for _X, since holes propagate over those
                        }
                    }
                }
            }
            return actions;
        }
    };


    /*
    // TODO: remove this after the switch to tagged union is done
    template <psize_t H, psize_t W>
    using SBPuzzle = SBPuzzleWHole<H, W>;
    */

    // Finally, we have to use a tagged union wrapper over the classes for
    // run-time polymorphic behavior, without actually using virtuals.
    // This is necessary because groups are run-time input and thus the
    // decision of which class to instantiate exactly should be done at run-time.
    template <psize_t H, psize_t W>
    class SBPuzzle {
    public:
        SBPuzzle(const array<uint8_t, H*W> &tiles) : tag(TypeTag::W_HOLE), puzzle(tiles) {}
        SBPuzzle(const array<uint8_t, H*W> &tiles, 
                 const array<bool, H*W> &mask) 
            : tag(TypeTag::NO_HOLE), puzzle(tiles, mask) 
        {
            // slightly more complicated, have to make a decision
            constexpr auto hole = H*W-1;
            if(mask[hole]) {
                std::cout << "Created whole\n";
                tag = TypeTag::W_HOLE;
                puzzle = SBPuzzleWHole<H, W>(tiles, mask);
            }
        }

        SBPuzzle(const SBPuzzle &) = default;
        SBPuzzle(SBPuzzle &&) = default;
        SBPuzzle &operator=(const SBPuzzle &) = default;
        SBPuzzle &operator=(SBPuzzle &&) = default;

        SBPuzzle(const SBPuzzleWHole<H, W> &o) : tag(TypeTag::W_HOLE), puzzle(o) {}
        SBPuzzle(const SBPuzzleNoHole<H, W> &o) : tag(TypeTag::NO_HOLE), puzzle(o) {}

        bool is_solved() const {
            switch(tag) {
                case TypeTag::W_HOLE:   return puzzle.w.is_solved();
                case TypeTag::NO_HOLE:  return puzzle.n.is_solved();
                default: throw_tterror(); return false;
            }
        }

        bool operator==(const SBPuzzle &other) const {
            if(tag != other.tag)
                throw std::invalid_argument("Puzzle type tags do not match in operator==");
            switch(tag) {
                case TypeTag::W_HOLE:   return puzzle.w == other.puzzle.w;
                case TypeTag::NO_HOLE:  return puzzle.n == other.puzzle.n;
                default: throw_tterror(); return false;
            }
        }

        size_t hash() const {
            switch(tag) {
                case TypeTag::W_HOLE:   return puzzle.w.hash();
                case TypeTag::NO_HOLE:  return puzzle.n.hash();
                default: throw_tterror(); return 0;
            }
        }

        template <psize_t HH, psize_t WW>
        friend std::ostream &operator<<(std::ostream &s, const SBPuzzle<HH, WW> &p) {
            switch(p.tag) {
                case TypeTag::W_HOLE:   return s << p.puzzle.w;
                case TypeTag::NO_HOLE:  return s << p.puzzle.n;
                default: throw_tterror(); return s;
            }
        }

        int manhattan_distance_to_solution() const {
            switch(tag) {
                case TypeTag::W_HOLE:   return puzzle.w.manhattan_distance_to_solution();
                case TypeTag::NO_HOLE:  return puzzle.n.manhattan_distance_to_solution();
                default: throw_tterror(); return 0;
            }
        }

        uint8_t lookup_cost(const PDB<H, W> *pdb) const {
            switch(tag) {
                case TypeTag::W_HOLE:   return puzzle.w.lookup_cost(pdb);
                case TypeTag::NO_HOLE:  return puzzle.n.lookup_cost(pdb);
                default: throw_tterror(); return 0;
            }
        }

        template <typename PDBType>
        size_t determine_index(uint8_t group_no, const PDBType &pdb) const {
            switch(tag) {
                case TypeTag::W_HOLE:   return puzzle.w.determine_index(group_no, pdb);
                case TypeTag::NO_HOLE:  return puzzle.n.determine_index(group_no, pdb);
                default: throw_tterror(); return 0;
            }
        }

        SBPuzzle goal_state() const {
            switch(tag) {
                case TypeTag::W_HOLE:   return SBPuzzle(puzzle.w.goal_state());
                case TypeTag::NO_HOLE:  return SBPuzzle(puzzle.n.goal_state());
                default: throw_tterror(); return SBPuzzle(puzzle.w.goal_state());
            }
        }

        uint64_t apply_action(TileSwapAction a) {
            switch(tag) {
                case TypeTag::W_HOLE:   return puzzle.w.apply_action(a);
                case TypeTag::NO_HOLE:  return puzzle.n.apply_action(a);
                default: throw_tterror(); return 0;
            }
        }


        template <class Action>
        std::vector<Action> possible_actions() const {
            return PADelegate<Action>::f(*this); // delegate to the struct
        }


    private:
        enum class TypeTag : uint8_t {
            W_HOLE,
            NO_HOLE
        } tag;
        union PuzzleUnion {
            SBPuzzleWHole<H, W> w;
            SBPuzzleNoHole<H, W> n;

            PuzzleUnion(const SBPuzzleWHole<H, W> &wh) : w(wh) {}
            PuzzleUnion(const SBPuzzleNoHole<H, W> &nh) : n(nh) {}
            PuzzleUnion(const array<uint8_t, H*W> &tiles) : w(tiles) {}
            // initialize nohole by default, change if not the case
            PuzzleUnion(const array<uint8_t, H*W> &tiles, 
                        const array<bool, H*W> &mask) : n(tiles, mask) {} 
        } puzzle;

        template <typename Action, typename Dummy = void>
        struct PADelegate {
            static std::vector<Action> f(const SBPuzzle &p);
        };

        static void throw_tterror() {
            throw std::runtime_error("Invalid type tag");
        }
    };

    template <psize_t H, psize_t W>
    template <typename Dummy>
    struct SBPuzzle<H, W>::PADelegate<TileSwapAction, Dummy> {
        static auto f(const SBPuzzle<H, W> &p) {
            switch(p.tag) {
                case SBPuzzle<H, W>::TypeTag::W_HOLE:   
                    return p.puzzle.w.template possible_actions<TileSwapAction>();
                case SBPuzzle<H, W>::TypeTag::NO_HOLE:  
                    return p.puzzle.n.template possible_actions<TileSwapAction>();
            }
            return p.puzzle.n.template possible_actions<TileSwapAction>();
        }
    };

}
    

// add hashability
namespace std {
    template <sbpuzzle::psize_t H, sbpuzzle::psize_t W>
    struct hash<sbpuzzle::SBPuzzleWHole<H, W>> 
        : sbpuzzle::details::hash<sbpuzzle::SBPuzzleWHole<H, W>> {};

    template <sbpuzzle::psize_t H, sbpuzzle::psize_t W>
    struct hash<sbpuzzle::SBPuzzleNoHole<H, W>> 
        : sbpuzzle::details::hash<sbpuzzle::SBPuzzleNoHole<H, W>> {};
    
    template <sbpuzzle::psize_t H, sbpuzzle::psize_t W>
    struct hash<sbpuzzle::SBPuzzle<H, W>> 
        : sbpuzzle::details::hash<sbpuzzle::SBPuzzle<H, W>> {};
}

#endif
