#ifndef __DAVI_INPUT_GENERATOR_HPP__
#define __DAVI_INPUT_GENERATOR_HPP__

#include <iostream>
#include <vector>
#include <array>
#include <random>
#include <limits>
#include <string>
#include <memory>
#include <cmath>

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
            // distribution for picking moves, lcm of 2, 3 and 4
            static std::uniform_int_distribution<int64_t> ad2(0, 1);
            static std::uniform_int_distribution<int64_t> ad3(0, 2);
            static std::uniform_int_distribution<int64_t> ad4(0, 3);
            // fill a constant tile array with goal state values
            std::array<uint8_t, H*W> goal_tiles;
            std::iota(goal_tiles.begin(), goal_tiles.end(), 0);
            // std::cout << "MDM: " << move_distribution.max() << std::endl;
            for(size_t i = 0; i < num_puzzles; ++i) {
                // create a goal state puzzle and select a number of moves
                std::array<uint8_t, H*W> tiles = goal_tiles;
                int64_t num_moves = move_distribution(rng);
                // randomly apply a move to the puzzle num_moves times
                // first, generate num_moves valid moves
                uint8_t hp = HOLE<H, W>;
                uint8_t prev_action_inverse_index = -1;
                for(int64_t j = 0; j < num_moves; ++j) {
                    const uint8_t *valid_actions = tiles_get_valid_moves<H, W>(hp);
                    uint8_t num_valid = valid_actions[4];
                    // select a random action index
                    uint8_t random_index;
                    switch(num_valid) {
                        case 2: random_index = ad2(rng); break;
                        case 3: random_index = ad3(rng); break;
                        case 4: random_index = ad4(rng); break;
                        default: throw std::runtime_error("Unexpected amount of valid moves");
                    }
                    // if the inverse action was previously done, select the next one instead
                    random_index = (prev_action_inverse_index == valid_actions[random_index]) ?
                                   random_index + 1 : random_index;
                    // cmove is faster than modulo with a variable
                    random_index = (random_index == num_valid) ? 0 : random_index;
                    uint8_t action_index = valid_actions[random_index];
                    // compute the tile position and apply the move
                    uint8_t tp = hp + OFFSETS<W>[action_index];
                    tiles[hp] = tiles[tp];
                    hp = tp; // update hole position
                    prev_action_inverse_index = inv_action_index(action_index);
                }
                tiles[hp] = HOLE<H, W>;
                // now that the action sequence is ready, apply it to the puzzle
                /*
                // debugging code
                for(auto a : action_sequence)
                    std::cout << ((int)a.tpos) << "-" << ((int)a.hpos) << " ";
                std::cout << std::endl;
                std::cout << puzzle << std::endl;
                */
                // add the puzzle
                if(num_moves_out != nullptr)
                    (*num_moves_out)[i] = num_moves;
                puzzles_out[i] = SBPuzzle<H, W>(tiles);
            }
        }

        // generate a weighted discrete distribution
        // weight by sqrt(scramble_max - scramble_min + 1)
        template <typename OutputIterator>
        void fill_scrambling_weights(int64_t min, int64_t max, OutputIterator out) {
            int64_t size = max - min + 1;
            double sum = 0.0;
            OutputIterator itr = out;
            std::vector<double> unnormalized_weights(size);
            for(int64_t i = 0; i < size; ++i) 
                // sum += (unnormalized_weights[i] = 1.0/size);//sqrt(i + min));
                sum += (unnormalized_weights[i] = sqrt(i + min));//sqrt(i + min));
            for(int64_t i = 0; i < size; ++i)
                *itr++ = unnormalized_weights[i] / sum;
        }

        template <typename IntType>
        class OffsetDiscreteDistribution {
        public:
            template <typename InputItr>
            OffsetDiscreteDistribution(InputItr wstart, InputItr wend, IntType off) 
                : distr(wstart, wend), offset(off) {}

            template <typename RNG>
            IntType operator()(RNG &rng) {
                return distr(rng) + offset;
            }

        private:
            std::discrete_distribution<IntType> distr;
            IntType offset;
        };

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
            // try a weighted distribution instead of uniform
            // weight by sqrt(scramble_max - scramble_min + 1)
            std::vector<double> weights;
            details::fill_scrambling_weights(scramble_min, scramble_max, 
                                             std::back_inserter(weights));
            details::OffsetDiscreteDistribution<int64_t> 
                move_distr(weights.begin(), weights.end(), scramble_min);
            details::fill_scrambled_puzzles<H, W>(batch_size, move_distr, rng, puzzles);
            // generate possible actions for each puzzle and mark the max number of moves
            max_action_index = 0;
            for(size_t i = 0; i < batch_size; ++i) {
                puzzle_actions[i] = puzzles[i].template action_generator<TileSwapAction>();
                size_t asize = puzzle_actions[i].size();
                if(max_action_index < asize)
                    max_action_index = asize;
            }
            action_index = 0; // set action index to zero
        }

        // C-style output parameter for interfacing
        void get_batch_states(void *out, bool *out_is_goal) const {
            #ifdef ONE_HOT
            float *out_encoded = static_cast<float *>(out);
            for(size_t i = 0; i < batch_size; ++i)
                one_hot_encode<uint8_t, float>(puzzles[i].get_tiles().data(), PUZZLE_SIZE,
                                               &out_encoded[i * ONE_HOT_ENCODED_SIZE]);
            #else 
            uint8_t *out_states = static_cast<uint8_t *>(out);
            for(size_t i = 0; i < batch_size; ++i)
                std::copy(puzzles[i].get_tiles().begin(), puzzles[i].get_tiles().end(), &out_states[i * PUZZLE_SIZE]);
            #endif
            std::transform(puzzles.begin(), puzzles.end(), out_is_goal,
                           [](const SBPuzzle<H, W> &p) -> bool { return p == GOAL_STATE; });
            /*
            for(size_t i = 0; i < batch_size; ++i) {
                std::cout << puzzles[i] << std::endl;
                for(size_t j = i * PUZZLE_SIZE; j < (i+1)*PUZZLE_SIZE; ++j) {
                    std::cout << static_cast<int>(out_states[j]) << " ";
                }
                std::cout << std::endl;
            }
            for(size_t i = 0; i < batch_size; ++i)
                std::cout << out_is_goal[i] << " ";
            std::cout << std::endl;
            */
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
                SBPuzzle<H, W> p = puzzles[i];
                if(action_index < puzzle_actions[i].size()) { // neighbor exists
                    // create the neighbor, check if it a goal state
                    p.apply_action(puzzle_actions[i][action_index]);
                    if(p == GOAL_STATE)
                        out_is_goal[i] = true;
                } else { // mark as non-existant
                    out_exists[i] = false;
                }
                #ifdef ONE_HOT
                // one hot encode the neighbor
                float *out_encoded = static_cast<float *>(out);
                one_hot_encode(p.get_tiles().data(), PUZZLE_SIZE,
                               &out_encoded[i * ONE_HOT_ENCODED_SIZE]);
                #else
                uint8_t *out_states = static_cast<uint8_t *>(out);
                std::copy(p.get_tiles().begin(), p.get_tiles().end(), &out_states[i * PUZZLE_SIZE]);
                #endif
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
