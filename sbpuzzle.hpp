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

// TODO: consider alternatives to returning a vector for possible_actions()
// a custom iterator-like type would work nicely if it can be made light-weight

#include "sblock_utils.hpp"

namespace sbpuzzle {
        
    using std::array;

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
        static constexpr psize_t OFFSETS[] = {static_cast<psize_t>(-W), +1, +W, static_cast<psize_t>(-1)};
        // offset applied to moves allowed by tiles_mark_valid_moves

        template <psize_t H, psize_t W>
        static constexpr psize_t SIZE = H*W;

        template <psize_t H, psize_t W>
        static constexpr psize_t HOLE = H*W-1;
        
        enum class Direction : uint8_t {
            UP, RIGHT, DOWN, LEFT
        };

        std::array<Direction, 4> DIRECTION_REVERSE {
            Direction::DOWN,
            Direction::LEFT,
            Direction::UP,
            Direction::RIGHT
        };

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
    
    namespace details {
        template <psize_t H, psize_t W>
        inline void tiles_correct_fill(array<pcell_t, H*W> &tiles) {
            for(size_t size = tiles.size(), i = 0; i < size; ++i)
                tiles[i] = i;
        }

        template <psize_t H, psize_t W>
        inline void tiles_correct_fill(array<pcell_t, H*W> &tiles, 
                                       const array<bool, H*W> &mask) 
        {
            for(size_t size = tiles.size(), i = 0; i < size; ++i)
                tiles[i] = mask[i] ? i : _X;
        }

        template <psize_t H, psize_t W>
        inline void tiles_reconstruct_mask(const array<pcell_t, H*W> &tiles, 
                                           array<bool, H*W> &mask) 
        {
            std::fill(mask.begin(), mask.end(), false);
            for(size_t size = tiles.size(), i = 0; i < size; ++i)
                if(tiles[i] != _X)
                    mask[tiles[i]] = true;
        }

        template <psize_t H, psize_t W>
        inline bool tiles_in_correct_places(const array<pcell_t, H*W> &tiles) {
            for(size_t size = tiles.size(), i = 0; i < size; ++i) 
                if(tiles[i] != _X && tiles[i] != HOLE<H, W> && tiles[i] != i)
                    return false;
            return true;
        }

        template <psize_t H, psize_t W>
        inline bool tiles_equal(const array<pcell_t, H*W> &t1, const array<pcell_t, H*W> &t2) {
            return std::equal(t1.begin(), t1.end(), t2.begin());
        }

        template <psize_t H, psize_t W>
        size_t tiles_hash(const array<pcell_t, H*W> tiles) {
            return hash_byte_array(&tiles[0], H*W);
        }

        template <psize_t H, psize_t W>
        std::ostream& tiles_stream(
                std::ostream &s, 
                const array<pcell_t, H*W> &tiles) 
        {
            // custom comparator that does not care about the _X value, takes it as min
            auto cmp = [](pcell_t a, pcell_t b) -> bool {
                // _X (don't care) values are replaced by 0
                a = (a == _X) ? 0 : a;
                b = (b == _X) ? 0 : b;
                return a < b;
            };
            const pcell_t *max = std::max_element(tiles.begin(), tiles.end(), cmp);
            // TODO: this currently only works for pcell_t == uint8_t
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
        pcost_t tiles_count_misplaced(const array<pcell_t, H*W> &tiles) {
            pcost_t dist = 0;
            for(size_t size = tiles.size(), i = 0; i < size; ++i) 
                if(tiles[i] != _X && tiles[i] != HOLE<H, W> && tiles[i] != i)
                    ++dist;
            return dist;
        }

        template <psize_t H, psize_t W>
        pcost_t tiles_manhattan_distance_to_solution(const array<pcell_t, H*W> &tiles) {
            pcost_t dist = 0;
            for(size_t size = tiles.size(), i = 0; i < size; ++i) {
                if(tiles[i] != _X && tiles[i] != HOLE<H, W>) {
                    // open to optimisation, but no need
                    psize_t row = i / W, col = i % W;
                    psize_t actual_row = tiles[i] / W, actual_col = tiles[i] % W;
                    dist += abs(row - actual_row) + abs(col - actual_col);
                }
            }
            return dist;
        }

        template <psize_t H, psize_t W>
        void tiles_mark_valid_moves(psize_t p, array<bool, 4> &directions) {
            psize_t rem = p % W;
            directions[0] = p >= W; // up
            directions[1] = rem < W-1; // right
            directions[2] = p < SIZE<H, W> - W; // down
            directions[3] = rem > 0; // left
        }

        // optimized version of mark_valid_moves for random generation
        // pretty low-level
        template <psize_t H, psize_t W>
        const uint8_t *tiles_get_valid_moves(psize_t p) {
            // cached results, last value holds the size [2, 3, 4], 0 if uninit
            // other four values hold the results
            static uint8_t cached_results[SIZE<H, W>][5] = {};
            constexpr static psize_t W1 = W - 1;
            constexpr static psize_t SZW = SIZE<H, W> - W;
            if(cached_results[p][4] == 0) {
                uint8_t i = 0;
                uint8_t rem = p % W;
                if(p >= W)
                    cached_results[p][i++] = 0;
                if(rem < W1)
                    cached_results[p][i++] = 1;
                if(p < SZW)
                    cached_results[p][i++] = 2;
                if(rem > 0)
                    cached_results[p][i++] = 3;
                cached_results[p][4] = i; // store the size
            }
            uint8_t *cached_dirs = cached_results[p];
            return cached_dirs;
        }

        inline uint8_t inv_action_index(uint8_t i) {
            return (i + 2) & 3; // 0 -> 2, 1 -> 3, 2 -> 0, 3 -> 1
        }

        template <psize_t H, psize_t W>
        size_t count_inversions(const array<pcell_t, H*W> &tiles) {
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
        P goal_state(const array<pcell_t, H*W> &tiles) {
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
        void tiles_prop_hole(array<pcell_t, H*W> &tiles, psize_t i) {
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
        void tiles_reprop_hole(array<pcell_t, H*W> &tiles, psize_t new_hole_pos) {
            for(size_t i = 0, size = tiles.size(); i < size; ++i)
                if(tiles[i] == HOLE<H, W>)
                    tiles[i] = _X;
            tiles[new_hole_pos] = HOLE<H, W>;
            tiles_prop_hole<H, W>(tiles, new_hole_pos);
        }

        // collapse hole for extended index calculation by DPDB
        template <psize_t H, psize_t W>
        void tiles_collapse_hole(array<pcell_t, H*W> &tiles) {
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
                const std::array<pcell_t, H*W> &tiles, 
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
    bool tiles_solvable(const array<pcell_t, H*W> &tiles) {
        // see https://www.cs.bham.ac.uk/~mdr/teaching/modules04/java2/TilesSolvability.html
        size_t hole_pos = std::find(tiles.begin(), tiles.end(), details::HOLE<H, W>) 
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
                
                const std::array<pcell_t, H*W> &get_tiles() const {
                    return tiles;
                }

                bool is_solved() const {
                    return details::tiles_in_correct_places<H, W>(tiles);
                }

                bool operator==(const SBPuzzleBase &other) const {
                    return details::tiles_equal<H, W>(tiles, other.tiles);
                }

                // Actions are returned over a generator like class
                // template <typename Action>
                // class generator {
                //     class iterator { ... };
                //     generator(...)
                //     iterator begin() const {...}
                //     iterator end() const {...}
                //  };
                //  generator<Action> action_generator() const;
                //  See the SBPuzzle implementation for an example.

                size_t hash() const {
                    return details::tiles_hash<H, W>(tiles);
                }

                friend std::ostream &operator<<(std::ostream &s, const SBPuzzleBase<H, W> &p) {
                    return details::tiles_stream<H, W>(s, p.tiles);
                }

                std::ostream &to_binary_stream(std::ostream &s) const {
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

                int num_misplaced_tiles() const {
                    return details::tiles_count_misplaced<H, W>(tiles);
                }

                pcost_t apply_action(TileSwapAction a) {
                    tiles[a.hpos] = tiles[a.tpos]; // move tile over the hole
                    tiles[a.tpos] = HOLE; // tile is now the hole
                    hole_pos = a.tpos;
                    return 1; // cost is always unit
                }

                pcost_t apply_action(DirectionAction d) {
                    int8_t off = details::OFFSETS<W>[static_cast<uint8_t>(d.dir)];
                    uint8_t tile_pos = hole_pos + off;
                    return apply_action(TileSwapAction(tile_pos, hole_pos));
                }

                // a small optimized function for applying a sequence of actions
                void apply_action_sequence(const std::vector<TileSwapAction> &as) {
                    for(auto a : as)
                        tiles[a.hpos] = tiles[a.tpos];
                    tiles[as.back().tpos] = HOLE;
                    hole_pos = as.back().tpos;
                }

            protected:
                // only callable by derived classes
                SBPuzzleBase() : hole_pos(details::HOLE<H, W>) {
                    details::tiles_correct_fill<H, W>(tiles);
                }

                static constexpr auto SIZE = details::SIZE<H, W>;
                static constexpr auto HOLE = details::HOLE<H, W>;

                array<pcell_t, SIZE> tiles;
                psize_t hole_pos;
        };

    /*                                                           */

    // The derived class for the case where the hole is masked
    template <psize_t H, psize_t W>
    class SBPuzzle : public SBPuzzleBase<H, W> {
    public:

        SBPuzzle() : Base() {}

        explicit SBPuzzle(const array<pcell_t, H*W> &i_tiles) {
            for(size_t i = 0; i < Base::SIZE; ++i) {
                if(i_tiles[i] == Base::HOLE)
                    Base::hole_pos = i;
                Base::tiles[i] = i_tiles[i];
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
        class generator {
        public:
            class iterator {
            public:
                iterator(const Action *a) : actp(a) {}
                iterator& operator++() { ++actp; return *this; }
                iterator operator++(int) { auto ret = *this; ++actp; return ret; }
                bool operator==(iterator other) { return actp == other.actp; }
                bool operator!=(iterator other) { return actp != other.actp; }
                Action operator*() { return *actp; }

                using difference_type = ptrdiff_t;
                using value_type = Action;
                using pointer = const Action *;
                using reference = const Action &;
                using iterator_category = std::input_iterator_tag;
            private:
                const Action *actp;
            };

            generator(psize_t hp) : hole_pos(hp) {}
            iterator begin() const { 
                auto ptr = records[hole_pos].actions.data();
                return iterator(ptr); 
            }
            iterator end() const { 
                auto ptr = records[hole_pos].actions.data();
                return iterator(ptr + records[hole_pos].size); 
            }

        private:
            // the only necessary state is the hole position
            psize_t hole_pos;

            // there is also a static vector which holds pre-computed available moves
            struct Record {
                std::array<Action, 4> actions;
                psize_t size;
            };

            // build the vector of records during compile time
            // there are only H*W*4 entries, very little
            static std::vector<Record> _construct_records() {
                // for templating the width, tag dispatching
                constexpr static details::WTag<W> tag; 
                constexpr static uint8_t W1 = W - 1;
                constexpr static uint8_t SW = details::SIZE<H, W> - W;
                using Dir = details::Direction;
                std::vector<Record> recs(details::SIZE<H, W>);
                for(psize_t index = 0; index < details::SIZE<H, W>; ++index) {
                    Record &rec = recs[index];
                    psize_t j = 0;
                    psize_t rem = index % W;
                    if(index >= W)
                        rec.actions[j++] = Action(tag, index, Dir::UP);
                    if(rem < W1)
                        rec.actions[j++] = Action(tag, index, Dir::RIGHT);
                    if(index < SW)
                        rec.actions[j++] = Action(tag, index, Dir::DOWN);
                    if(rem > 0)
                        rec.actions[j++] = Action(tag, index, Dir::LEFT);
                    rec.size = j;
                }
                return recs;
            }

            const static inline std::vector<Record> records = _construct_records();

        };

        template <class Action>
        generator<Action> action_generator() const {
            return generator<Action>(Base::hole_pos);
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

        uint64_t apply_action(TileSwapAction a) {
            uint64_t cost = Base::apply_action(a);
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
