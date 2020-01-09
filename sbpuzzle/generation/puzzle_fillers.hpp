#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_GENERATION_PUZZLE_FILLERS_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_GENERATION_PUZZLE_FILLERS_HPP

#include "../basic.hpp"
#include "offset_discrete_distribution.hpp"

namespace denizmsayin::sblock::sbpuzzle::generation {
    // create puzzles by shuffling tiles randomly
    template <psize_t H, psize_t W, class URNG, class OutputIterator>
    void fill_shuffled_puzzles(size_t num_puzzles, URNG &rng, OutputIterator out) {
        std::array<uint8_t, H*W> tiles;
        std::iota(tiles.begin(), tiles.end(), 0);
        for(size_t i = 0; i < num_puzzles; ++i) {
            do {
                std::shuffle(tiles.begin(), tiles.end(), rng); 
            } while(!sbpuzzle::tiles_solvable<H, W>(tiles));
            *out++ = Basic<H, W>(tiles);
        }
    }

    template <psize_t H, psize_t W, class URNG, class OutputIterator, class Distribution>
    void fill_scrambled_puzzles(size_t num_puzzles,
                                URNG &rng,
                                OutputIterator puzzles_out,
                                Distribution &move_distribution)
    {
        // distribution for picking moves, lcm of 2, 3 and 4
        // much faster than making a single 0-11 and taking modulo
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
            uint8_t hp = details::HOLE<H, W>;
            uint8_t prev_action_inverse_index = -1;
            for(int64_t j = 0; j < num_moves; ++j) {
                const uint8_t *valid_actions = details::tiles_get_valid_moves<H, W>(hp);
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
                uint8_t tp = hp + details::OFFSETS<W>[action_index];
                tiles[hp] = tiles[tp];
                hp = tp; // update hole position
                prev_action_inverse_index = details::inv_action_index(action_index);
            }
            tiles[hp] = details::HOLE<H, W>;
            // now that the action sequence is ready, apply it to the puzzle
            /*
            // debugging code
            for(auto a : action_sequence)
                std::cout << ((int)a.tpos) << "-" << ((int)a.hpos) << " ";
            std::cout << std::endl;
            std::cout << puzzle << std::endl;
            */
            // add the puzzle
            // if(num_moves_out != nullptr)
            //    (*num_moves_out)[i] = num_moves;
            *puzzles_out++ = Basic<H, W>(tiles);
        }
    }
}
#endif
