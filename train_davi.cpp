#include <iostream>
#include <vector>
#include <array>
#include <random>

#include "sbpuzzle.hpp"
#include "search2.hpp"
#include "heuristics.hpp"

#ifndef __H
#define __H 3
#endif
#ifndef __W
#define __W 3
#endif

using sbpuzzle::psize_t;

constexpr psize_t H = __H, W = __W;

using Puzzle = sbpuzzle::SBPuzzle<H, W>;
using TSA = sbpuzzle::TileSwapAction;
using ManhH = sbpuzzle::ManhattanHeuristic<H, W>;

template <psize_t H, psize_t W, class URNG, class Distribution>
void fill_scrambled_puzzles(size_t num_puzzles, 
                            Distribution &move_distribution,
                            URNG &rng,
                            std::vector<Puzzle> &puzzles_out,
                            std::vector<size_t> &num_moves_out)
{
    // fill a constant tile array with goal state values
    std::array<uint8_t, H*W> goal_tiles;
    std::iota(goal_tiles.begin(), goal_tiles.end(), 0);
    for(size_t i = 0; i < num_puzzles; ++i) {
        // create a goal state puzzle and select a number of moves
        Puzzle puzzle(goal_tiles);
        int64_t num_moves = move_distribution(rng);
        // randomly apply a move to the puzzle num_moves times
        for(int64_t j = 0; j < num_moves; ++j) {
            // TODO: perhaps prevent backtracking of previous move? might help just a little
            std::vector<TSA> actions = puzzle.template possible_actions<TSA>();
            std::uniform_int_distribution<int64_t> action_index_distr(0, actions.size()-1);
            int64_t action_i = action_index_distr(rng);
            puzzle.apply_action(actions[action_i]);
        }
        // add the puzzle
        num_moves_out.emplace_back(num_moves);
        puzzles_out.emplace_back(puzzle);
    }
}


int main() {
    auto rng = std::default_random_engine(42);
    std::uniform_int_distribution<int64_t> move_distr(1, 1000);
    std::vector<Puzzle> puzzles;
    std::vector<size_t> nmoves;

    size_t num_puzzles = 1000;

    fill_scrambled_puzzles<H, W>(num_puzzles, move_distr, rng, puzzles, nmoves);

    for(size_t i = 0; i < num_puzzles; ++i) {
        auto result = search2::a_star_search<Puzzle, TSA, ManhH>(puzzles[i]);
        std::cout << "Scrambles: " << nmoves[i] << ", Distance: " << result.cost << std::endl;
    }

    return 0;
}
