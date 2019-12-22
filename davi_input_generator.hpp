#ifndef __DAVI_INPUT_GENERATOR_HPP__
#define __DAVI_INPUT_GENERATOR_HPP__

#include <vector>
#include <array>
#include <random>
#include <limits>
#include <string>
#include <memory>

#include "sbpuzzle.hpp"
#include "sblock_utils.hpp"

// Goal: generate input for models that will be trained in pytorch
// Inputs: batch_size, scramble_min, scramble_max, action_i
// Outputs: states -> (batch_size, (H*W)**2)
//          neighbors -> (batch_size, (H*W)**2)
//          neighbor_exists -> (batch_size,)
//          is_goal_state -> (batch_size,)
// Since the python script will not do any puzzle related operations,
// all neighbor states have to be created here, as well as boolean
// arrays marking whether the neighbor exists and is a goal state

namespace sbpuzzle {
    namespace details {

        template <psize_t H, psize_t W, class URNG, class Distribution>
        static void fill_scrambled_puzzles(size_t num_puzzles,
                                    Distribution &move_distribution,
                                    URNG &rng,
                                    std::vector<SBPuzzle<H, W>> &puzzles_out, // prellocated
                                    std::vector<size_t> *num_moves_out = nullptr) // preallocated
        {
            using TSA = TileSwapAction;
            // fill a constant tile array with goal state values
            std::array<uint8_t, H*W> goal_tiles;
            std::iota(goal_tiles.begin(), goal_tiles.end(), 0);
            for(size_t i = 0; i < num_puzzles; ++i) {
                // create a goal state puzzle and select a number of moves
                SBPuzzle<H, W> puzzle(goal_tiles);
                int64_t num_moves = move_distribution(rng);
                // randomly apply a move to the puzzle num_moves times
                for(int64_t j = 0; j < num_moves; ++j) {
                    // TODO: perhaps prevent backtracking of previous move? 
                    // might help just a little
                    std::vector<TSA> actions = puzzle.template possible_actions<TSA>();
                    std::uniform_int_distribution<int64_t> 
                        action_index_distr(0, actions.size()-1);
                    int64_t action_i = action_index_distr(rng);
                    puzzle.apply_action(actions[action_i]);
                }
                // add the puzzle
                if(num_moves_out != nullptr)
                    (*num_moves_out)[i] = num_moves;
                puzzles_out[i] = puzzle;
            }
        }
    }

    template <psize_t H, psize_t W>
    class DaviInputGenerator {
    public:
        // initialize with preallocated buffers
        DaviInputGenerator(unsigned seed, size_t o_batch_size) : rng(seed), 
            batch_size(o_batch_size), puzzles(o_batch_size), 
            puzzle_actions(o_batch_size), action_index(0), max_action_index(0) {}

        void init_batch(int64_t scramble_min, int64_t scramble_max) {
            // create a set of scrambled puzzles
            std::uniform_int_distribution<int64_t> move_distr(scramble_min, scramble_max);
            details::fill_scrambled_puzzles<H, W>(batch_size, move_distr, rng, puzzles);
            // generate possible actions for each puzzle and mark the max number of moves
            max_action_index = 0;
            for(size_t i = 0; i < batch_size; ++i) {
                puzzle_actions[i] = puzzles[i].template possible_actions<TileSwapAction>();
                size_t asize = puzzle_actions[i].size();
                if(max_action_index < asize)
                    max_action_index = asize;
            }
            action_index = 0; // set action index to zero
        }

        // C-style output parameter for interfacing
        void get_batch_states(float *out_encoded) const {
            for(size_t i = 0; i < batch_size; ++i)
                one_hot_encode<uint8_t, float>(puzzles[i].get_tiles().data(), PUZZLE_SIZE,
                                               &out_encoded[i * ONE_HOT_ENCODED_SIZE]);
        }

        bool has_more_neighbors() const {
            return action_index < max_action_index;
        }
            
        void get_next_neighbors(float *out_encoded,
                                bool *out_exists,
                                bool *out_is_goal)
        {
            std::fill(out_exists, out_exists + batch_size, true);
            std::fill(out_is_goal, out_is_goal + batch_size, false);
            for(size_t i = 0; i < batch_size; ++i) {
                SBPuzzle<H, W> p = puzzles[i];
                if(action_index < puzzle_actions[i].size()) { // neighbor exists
                    // create the neighbor, check if it a goal state
                    p.apply_action(puzzle_actions[i][action_index]);
                    if(p == GOAL_STATE)
                        out_is_goal[i] = true;
                } else { // mark as non-existant
                    out_exists[i] = false;
                }
                // one hot encode the neighbor
                one_hot_encode(p.get_tiles().data(), PUZZLE_SIZE,
                               &out_encoded[i * ONE_HOT_ENCODED_SIZE]);
            }
            ++action_index;
        }

    private:
        std::default_random_engine rng;
        size_t batch_size;
        std::vector<SBPuzzle<H, W>> puzzles;
        std::vector<std::vector<TileSwapAction>> puzzle_actions;
        size_t action_index;
        size_t max_action_index;
        static const SBPuzzle<H, W> GOAL_STATE;
        static const size_t PUZZLE_SIZE;
        static const size_t ONE_HOT_ENCODED_SIZE;
    };

    template <psize_t H, psize_t W>
    const SBPuzzle<H, W> DaviInputGenerator<H, W>::GOAL_STATE = 
        SBPuzzle<H, W>::no_mask_goal_state();

    template <psize_t H, psize_t W>
    const size_t DaviInputGenerator<H, W>::PUZZLE_SIZE = H*W;

    template <psize_t H, psize_t W>
    const size_t DaviInputGenerator<H, W>::ONE_HOT_ENCODED_SIZE = H*H*W*W;
}

#endif
