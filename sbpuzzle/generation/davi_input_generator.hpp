#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_GENERATION_DAVI_INPUT_GENERATOR_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_GENERATION_DAVI_INPUT_GENERATOR_HPP

#include <iostream>
#include <vector>
#include <array>
#include <random>
#include <limits>
#include <string>
#include <memory>
#include <cmath>

#include "../basic.hpp"
#include "../../utils.hpp"
#include "factory.hpp"

// Goal: generate input for models that will be trained in pytorch
// Inputs: batch_size, scramble_min, scramble_max, action_i
// Outputs: states -> (batch_size, H*W)
//          neighbors -> (batch_size, H*W)
//          neighbor_exists -> (batch_size,)
//          is_goal_state -> (batch_size,)
// Since the python script will not do any puzzle related operations,
// all neighbor states have to be created here, as well as boolean
// arrays marking whether the neighbor exists and is a goal state

namespace denizmsayin::sblock::sbpuzzle::generation {

    template <psize_t H, psize_t W>
    class DaviInputGenerator {
    private:
        typedef std::default_random_engine rng_t;

    public:
        // initialize with preallocated buffers
        DaviInputGenerator(unsigned seed, size_t o_batch_size, RandomGeneratorType gen_t) 
            : gen_type(gen_t), rng(seed), batch_size(o_batch_size), 
              puzzles(o_batch_size, Basic<H, W>::uninitialized()), 
              puzzle_actions(o_batch_size), action_index(0), max_action_index(0) {}

        DaviInputGenerator(unsigned seed, size_t o_batch_size)
            : DaviInputGenerator(seed, o_batch_size, RandomGeneratorType::UNIFORM) {}

        DaviInputGenerator(unsigned seed, size_t o_batch_size, const std::string &gen_t_str)
            : DaviInputGenerator(seed, o_batch_size, str2randomgeneratortype(gen_t_str)) {}

        void init_batch(int64_t scramble_min, int64_t scramble_max) {
            // create a set of scrambled puzzles
            // try a weighted distribution instead of uniform
            // weight by sqrt(scramble_max - scramble_min + 1)
            typedef typename std::vector<Basic<H, W>>::iterator oitr_t;
            auto batch_generator = random_sbpuzzle_generator_factory<H, W, rng_t, oitr_t>(
                    gen_type, scramble_min, scramble_max);
            batch_generator(batch_size, rng, puzzles.begin());
            // generate possible actions for each puzzle and mark the max number of moves
            max_action_index = 0;
            for(size_t i = 0; i < batch_size; ++i) {
                // TODO: improve this part
                std::vector<TileSwapAction> tmp;
                for(auto action : puzzles[i].template action_generator<TileSwapAction>())
                    tmp.push_back(action);
                puzzle_actions[i] = tmp; 
                size_t asize = puzzle_actions[i].size();
                if(max_action_index < asize)
                    max_action_index = asize;
            }
            action_index = 0; // set action index to zero
        }

        // C-style output parameter for interfacing
        void get_batch_states(void *out, bool *out_is_goal) const {
            pcell_t *out_states = static_cast<pcell_t *>(out);
            for(size_t i = 0; i < batch_size; ++i) {
                std::copy(puzzles[i].get_tiles().begin(), 
                          puzzles[i].get_tiles().end(), 
                          &out_states[i * PUZZLE_SIZE]);
            }

            std::transform(puzzles.begin(), puzzles.end(), out_is_goal,
                           [](const Basic<H, W> &p) -> bool { return p == GOAL_STATE; });
        }

        bool has_more_neighbors() const {
            return action_index < max_action_index;
        }
            
        void get_next_neighbors(void *out,
                                bool *out_exists,
                                bool *out_is_goal)
        {
            std::fill(out_exists, out_exists + batch_size, true);
            std::fill(out_is_goal, out_is_goal + batch_size, false);
            for(size_t i = 0; i < batch_size; ++i) {
                Basic<H, W> p = puzzles[i];
                if(action_index < puzzle_actions[i].size()) { // neighbor exists
                    // create the neighbor, check if it a goal state
                    p.apply_action(puzzle_actions[i][action_index]);
                    if(p == GOAL_STATE)
                        out_is_goal[i] = true;
                } else { // mark as non-existant
                    out_exists[i] = false;
                }
                pcell_t *out_states = static_cast<pcell_t *>(out);
                std::copy(p.get_tiles().begin(), 
                          p.get_tiles().end(), 
                          &out_states[i * PUZZLE_SIZE]);
            }
            ++action_index;
        }

    private:
        RandomGeneratorType gen_type;
        rng_t rng;
        size_t batch_size;
        std::vector<Basic<H, W>> puzzles;
        std::vector<std::vector<TileSwapAction>> puzzle_actions;
        size_t action_index;
        size_t max_action_index;
        static const Basic<H, W> GOAL_STATE;
        static const size_t PUZZLE_SIZE;
    };

    template <psize_t H, psize_t W>
    const Basic<H, W> DaviInputGenerator<H, W>::GOAL_STATE = 
        Basic<H, W>::goal_state();

    template <psize_t H, psize_t W>
    const size_t DaviInputGenerator<H, W>::PUZZLE_SIZE = H*W;

}

#endif
