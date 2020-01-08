#ifndef __SBPUZZLE_TILES_HPP__
#define __SBPUZZLE_TILES_HPP__

#include <array>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include "defs.hpp"

// TODO: cleanup

namespace sbpuzzle {
    namespace details {
        using std::array;

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

        // ZOBRIST HASHING: a cool method for hashing game tables
        // each (square, value) pair has a bitstring associated with it
        // in the sbpuzzle case, we have H*W tiles & H*W values
        
        unsigned ZOBRIST_SEED = 509;

        template <psize_t H, psize_t W>
        std::vector<size_t> ZOBRISTS = generate_different_bitstrings(SIZE<H, W> * SIZE<H, W>, 
                                                                     ZOBRIST_SEED);

        template <psize_t H, psize_t W>
        size_t get_zobrist(pcell_t pos, pcell_t value) {
            return ZOBRISTS<H, W>[pos * SIZE<H, W> + value];
        }

        template <psize_t H, psize_t W>
        size_t tiles_zobrist_hash(const array<pcell_t, H*W> tiles) {
            size_t h = 0;
            for(size_t i = 0, size = tiles.size(); i < size; ++i)
                h ^= get_zobrist(i, tiles[i]);
            return h;
        }

        template <psize_t H, psize_t W>
        size_t tiles_hash(const array<pcell_t, H*W> tiles) {
            return hash_byte_array(tiles.data(), H*W);
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


        // construct a static vector which holds pre-computed available moves
        // from each position. The pre-computation is inexpensive because there
        // are only H*W*4 values to calculate/store
        struct Record {
            std::array<Direction, 4> dirs;
            uint8_t size;
        };

        // build the vector of records during compile time
        // there are only H*W*4 entries, very little
        template <psize_t H, psize_t W>
        static std::vector<Record> _construct_records() {
            // for templating the width, tag dispatching
            constexpr static uint8_t W1 = W - 1;
            constexpr static uint8_t SW = details::SIZE<H, W> - W;
            using Dir = details::Direction;
            std::vector<Record> recs(details::SIZE<H, W>);
            for(psize_t index = 0; index < details::SIZE<H, W>; ++index) {
                Record &rec = recs[index];
                psize_t j = 0;
                psize_t rem = index % W;
                if(index >= W)
                    rec.dirs[j++] = Dir::UP;
                if(rem < W1)
                    rec.dirs[j++] = Dir::RIGHT;
                if(index < SW)
                    rec.dirs[j++] = Dir::DOWN;
                if(rem > 0)
                    rec.dirs[j++] = Dir::LEFT;
                rec.size = j;
            }
            return recs;
        }

        template <psize_t H, psize_t W>
        const std::vector<Record> DIRECTION_RECORDS = _construct_records<H, W>();

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
            const Record &rec = DIRECTION_RECORDS<H, W>[i];
            for(auto j = 0; j < rec.size; ++j) {
                auto np = i + OFFSETS<W>[static_cast<uint8_t>(rec.dirs[j])];
                if(tiles[np] == _X) {
                    tiles[np] = HOLE<H, W>;
                    tiles_prop_hole<H, W>(tiles, np);
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

    // a utility function to decide if a set of tiles is solvable
    template <psize_t H, psize_t W>
    bool tiles_solvable(const std::array<pcell_t, H*W> &tiles) {
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

}


#endif
