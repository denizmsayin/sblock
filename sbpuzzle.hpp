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
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>

// TODO: consider alternatives to returning a vector for possible_actions()
// a custom iterator-like type would work nicely if it can be made light-weight

namespace sbpuzzle {

    struct TileSwapAction {
        uint8_t tpos, hpos;

        TileSwapAction(uint8_t t, uint8_t h) : tpos(t), hpos(h) {}

        TileSwapAction inverse(TileSwapAction a) {
            return TileSwapAction(a.hpos, a.tpos);
        }
    };

    namespace details {
        typedef int psize_t;
        
        constexpr uint8_t _X = 255;

        template <psize_t H, psize_t W>
        inline void tiles_correct_fill(uint8_t tiles[]) {
            constexpr auto size = H*W;
            for(auto i = 0; i < size; ++i)
                tiles[i] = i;
        }

        template <psize_t H, psize_t W>
        inline void tiles_correct_fill(uint8_t tiles[], const bool mask[]) {
            constexpr auto size = H*W;
            for(auto i = 0; i < size; ++i)
                tiles[i] = mask[i] ? i : _X;
        }

        template <psize_t H, psize_t W>
        inline void tiles_reconstruct_mask(const uint8_t tiles[], bool mask[]) {
            constexpr auto size = H*W;
            std::fill(mask, mask + size, false);
            for(auto i = 0; i < size; ++i)
                if(tiles[i] != _X)
                    mask[tiles[i]] = true;
        }

        template <psize_t H, psize_t W>
        inline bool tiles_in_correct_places(const uint8_t tiles[]) {
            constexpr auto size = H*W;
            for(auto i = 0; i < size; ++i) 
                if(tiles[i] != _X && tiles[i] != i)
                    return false;
            return true;
        }

        template <psize_t H, psize_t W>
        inline bool tiles_equal(const uint8_t t1[], const uint8_t t2[]) {
            constexpr auto size = H*W;
            return std::equal(t1, t1+size, t2);
        }

        // copied from the boost implementation
        inline size_t hash_combine(size_t h1, size_t h2) {
            return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        }

        template <typename P>
        inline size_t cast_hash(const void *p) {
            return std::hash<P>()(*static_cast<const P *>(p));
        }

        inline const void *offset_voidptr(const void *p, size_t x) {
            return static_cast<const void *>(reinterpret_cast<const char *>(p) + x);
        }

        template <psize_t H, psize_t W>
        size_t tiles_hash(const uint8_t tiles[]) {
            constexpr auto size = H*W;
            // first, do all you can using 8 bytes each time
            size_t seed = 0;
            constexpr std::hash<uint64_t> hasher64;
            auto n = size;
            const uint64_t *itr = reinterpret_cast<const uint64_t *>(tiles);
            while(n >= 8) {
                seed = hash_combine(seed, hasher64(*itr++));
                n -= 8;
            }
            // hash the remainder
            const void *p = reinterpret_cast<const void *>(itr);
            switch(size) {
                case 1: 
                    seed = hash_combine(seed, cast_hash<uint8_t>(p)); 
                    break;
                case 2: 
                    seed = hash_combine(seed, cast_hash<uint16_t>(p)); 
                    break;
                case 3: 
                    seed = hash_combine(hash_combine(seed, cast_hash<uint16_t>(p)),
                                        cast_hash<uint8_t>(offset_voidptr(p, 2)));
                    break;
                case 4: 
                    seed = hash_combine(seed, cast_hash<uint32_t>(p)); 
                    break;
                case 5: 
                    seed = hash_combine(hash_combine(seed, cast_hash<uint32_t>(p)),
                                        cast_hash<uint8_t>(offset_voidptr(p, 4)));
                    break;
                case 6:
                    seed = hash_combine(hash_combine(seed, cast_hash<uint32_t>(p)),
                                        cast_hash<uint16_t>(offset_voidptr(p, 4)));
                    break;
                case 7:
                    seed = hash_combine(hash_combine(hash_combine(seed, cast_hash<uint32_t>(p)),
                                                     cast_hash<uint16_t>(offset_voidptr(p, 4))),
                                        cast_hash<uint8_t>(offset_voidptr(p, 6)));
                    break;
                default: ;
            }
            return seed;
            // brought form 34 ms to 28 ms compared to hashing one by one
        }

        template <psize_t H, psize_t W>
        std::ostream& tiles_stream(
                std::ostream &s, 
                const uint8_t tiles[]) 
        {
            constexpr auto size = H*W;
            // custom comparator that does not care about the _X value, takes it as min
            constexpr auto cmp = [](uint8_t a, uint8_t b) -> bool {
                // _X (don't care) values are replaced by 0
                a = (a == _X) ? 0 : a;
                b = (b == _X) ? 0 : b;
                return a < b;
            };
            const uint8_t *max = std::max_element(tiles, tiles + size, cmp);
            uint8_t num_digits = (*max > 99) ? 3 : ((*max > 9) ? 2 : 1); 
            int num_dashes = W * (num_digits + 3) + 1;
            std::string dash_str(num_dashes, '-');
            std::string empty_str(num_digits - 1, ' ');
            for(auto i = 0, k = 0; i < H; i++) {
                s << dash_str << std::endl;
                for(auto j = 0; j < W; j++) {
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
        uint64_t tiles_manhattan_distance_to_solution(const uint8_t tiles[]) {
            constexpr auto size = H*W;
            constexpr auto hole = size-1;
            uint64_t dist = 0;
            for(auto i = 0; i < size; ++i) {
                if(tiles[i] != _X && tiles[i] != hole) {
                    // open to optimisation, but no need
                    uint8_t row = i / W, col = i % W;
                    uint8_t actual_row = tiles[i] / W, actual_col = tiles[i] % W;
                    dist += abs(row - actual_row) + abs(col - actual_col);
                }
            }
            return dist;
        }

        template <psize_t H, psize_t W>
        void tiles_mark_valid_moves(uint8_t p, bool directions[]) {
            constexpr auto size = H*W;
            int rem = p % W;
            directions[0] = p >= W; // up
            directions[1] = rem < W-1; // right
            directions[2] = p < size-W; // down
            directions[3] = rem > 0; // left
        }

        template <psize_t W>
        static constexpr int8_t OFFSETS[] = {-W, +1, +W, -1};
        // offset applied to moves allowed by tiles_mark_valid_moves

        template <typename RandomAccessIterator>
        int count_inversions(RandomAccessIterator begin, RandomAccessIterator end) {
            // count inversions with O(n^2) instead of mergesort style,
            // since the vectors are very small
            int inversions = 0;
            int size = end - begin;
            int hole = size - 1;
            for(RandomAccessIterator i = begin; i != end; i++) {
                if(*i != hole) {
                    for(RandomAccessIterator j = i+1; j != end; j++) {
                        if(*i > *j) {
                            inversions++;
                        }
                    }
                }
            }
            return inversions;
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
        P goal_state(const uint8_t tiles[]) {
            // this function should not be called often, so it seems
            // to me it makes more sense to reconstruct the mask array
            // here rather than store it in each puzzle instance
            constexpr auto size = H*W;
            bool mask[size];
            tiles_reconstruct_mask<H, W>(tiles, mask);
            uint8_t tiles2[size];
            tiles_correct_fill<H, W>(tiles2, mask);
            return P(tiles2, mask);
        }


    }

    // a utility function to decide if a set of tiles is solvable
    template <details::psize_t H, details::psize_t W>
    bool tiles_solvable(const uint8_t tiles[]) {
        constexpr auto size = H*W;
        constexpr uint8_t hole = H*W-1;
        // see https://www.cs.bham.ac.uk/~mdr/teaching/modules04/java2/TilesSolvability.html
        uint8_t hole_pos = std::find(tiles, tiles + size, hole) - tiles;
        int inversions = details::count_inversions(tiles, tiles + size);
        int hole_row_from_bottom = H - hole_pos % W;
        if(W % 2 == 1) // odd width
            return inversions % 2 == 0; // must have even inversions
        if(hole_row_from_bottom % 2 == 0) // even width
            return inversions % 2 == 1; // must have odd inversions
        // otherwise even
        return inversions % 2 == 1;
    }

    // I've elected to define the functions right in the declaration since
    // writing them separately takes time and I keep changing implementations...

    template <details::psize_t H, details::psize_t W>
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

        template <int HH, int WW>
        friend std::ostream &operator<<(std::ostream &s, const SBPuzzleBase<HH, WW> &p) {
            return details::tiles_stream<HH, WW>(s, p.tiles);
        }

        int manhattan_distance_to_solution() const {
            return details::tiles_manhattan_distance_to_solution<H, W>(tiles);
        }

    protected:
        // only callable by derived classes
        SBPuzzleBase() {}

        constexpr static auto SIZE = H*W;
        constexpr static uint8_t HOLE = H*W-1;
        
        uint8_t tiles[SIZE];

    };
 
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //

    template <details::psize_t H, details::psize_t W>
    class SBPuzzleWHole : public SBPuzzleBase<H, W> {
    public:

        explicit SBPuzzleWHole(const uint8_t i_tiles[]) {
            for(auto i = 0; i < Base::SIZE; ++i) {
                if(i_tiles[i] == Base::HOLE)
                    hole_pos = i;
                Base::tiles[i] = i_tiles[i];
            }
        }

        explicit SBPuzzleWHole(const uint8_t i_tiles[], const bool mask[]) {
            for(auto i = 0; i < Base::SIZE; ++i) {
                if(mask[i_tiles[i]]) {
                    if(i_tiles[i] == Base::HOLE)
                        hole_pos = i;
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

        uint64_t apply_action(TileSwapAction a) {
            Base::tiles[a.hpos] = Base::tiles[a.tpos]; // move tile over the hole
            Base::tiles[a.tpos] = Base::HOLE; // tile is now the hole
            hole_pos = a.tpos;
            return 1; // cost is always unit
        }

    private:
        using Base = SBPuzzleBase<H, W>;

        details::psize_t hole_pos;

        SBPuzzleWHole() {} // empty initialization for in-place business
       
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
    template <details::psize_t H, details::psize_t W>
    template <typename Dummy>
    struct SBPuzzleWHole<H, W>::PADelegate<TileSwapAction, Dummy> {
        static auto f(const SBPuzzleWHole<H, W> &p) {
            auto hp = p.hole_pos;
            std::vector<TileSwapAction> actions;
            bool valid[4];
            details::tiles_mark_valid_moves<H, W>(hp, valid);
            for(auto dir = 0; dir < 4; ++dir)
                if(valid[dir])
                    actions.emplace_back(hp + details::OFFSETS<W>[dir], hp);
            /*
            static int Q = 0;
            std::cout << p << std::endl;
            for(auto &a : actions)
                std::cout << '(' << ((int) a.tpos) << ", " << ((int) a.hpos) << ") ";
            std::cout << (Q++) << std::endl;
            std::cout << "**********************\n";
            */
            return actions;
        }
    };

    /*                                                           */
    /*                                                           */
    /*                                                           */
    /*                                                           */
    /*                                                           */
   
    template <details::psize_t H, details::psize_t W>
    class SBPuzzleNoHole : public SBPuzzleBase<H, W> {
    public:

        explicit SBPuzzleNoHole(const uint8_t i_tiles[], const bool mask[]) {
            for(auto i = 0; i < Base::SIZE; ++i) {
                if(mask[i_tiles[i]]) {
                    if(Base::tiles[i] == Base::HOLE)
                        throw std::invalid_argument("Constructing SBPuzzleNoHole with a masked hole will give invalid results");
                    Base::tiles[i] = i_tiles[i];
                } else {
                    Base::tiles[i] = details::_X;
                }
            }
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
            Base::tiles[a.hpos] = Base::tiles[a.tpos]; // move tile over X
            Base::tiles[a.tpos] = details::_X; // tile is now X
            return 1; // cost is always unit
        }
    
    private:
        using Base = SBPuzzleBase<H, W>;

        // TODO: consider caching hole/tile positions for faster searching

        template <typename Action, typename Dummy = void>
        struct PADelegate {
            static std::vector<Action> f(const SBPuzzleNoHole &p);
        };
    };

    // PADelegate specialization for the TileSwapAction class
    template <details::psize_t H, details::psize_t W>
    template <typename Dummy>
    struct SBPuzzleNoHole<H, W>::PADelegate<TileSwapAction, Dummy> {
        static auto f(const SBPuzzleNoHole<H, W> &p) {
            std::vector<TileSwapAction> actions;
            // while constructing disjoint pattern databases, usually there
            // will be more X values than actual tiles, so it makes more
            // sense to look around tiles rather than X values
            for(auto i = 0; i < p.SIZE; ++i) {
                if(p.tiles[i] != details::_X) { // only actual tiles
                    bool valid[4];
                    details::tiles_mark_valid_moves<H, W>(i, valid);
                    for(auto dir = 0; dir < 4; ++dir) { // check the 4-neighborhood
                        if(valid[dir]) {
                            auto np = i + details::OFFSETS<W>[dir]; // candidate swap position
                            if(p.tiles[np] == details::_X) // can swap if the neighbor is X
                                actions.emplace_back(i, np);
                        }
                    }
                }
            }
            /*
            static int Q = 0;
            std::cout << p << std::endl;
            for(auto &a : actions)
                std::cout << '(' << ((int) a.tpos) << ", " << ((int) a.hpos) << ") ";
            std::cout << (Q++) << std::endl;
            std::cout << "**********************\n";
            */
            return actions;
        }
    };

    // TODO: remove this after the switch to tagged union is done
    template <details::psize_t H, details::psize_t W>
    using SBPuzzle = SBPuzzleWHole<H, W>;

}
    

// add hashability
namespace std {
    template <sbpuzzle::details::psize_t H, sbpuzzle::details::psize_t W>
    struct hash<sbpuzzle::SBPuzzleWHole<H, W>> 
        : sbpuzzle::details::hash<sbpuzzle::SBPuzzleWHole<H, W>> {};

    template <sbpuzzle::details::psize_t H, sbpuzzle::details::psize_t W>
    struct hash<sbpuzzle::SBPuzzleNoHole<H, W>> 
        : sbpuzzle::details::hash<sbpuzzle::SBPuzzleNoHole<H, W>> {};
}

#endif
