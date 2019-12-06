#include <iostream>
#include <array>
#include <vector>
#include <queue>
#include <random>
#include <numeric>
#include <unordered_set>
#include <thread>
#include <mutex>

#include "sblock_utils.hpp"
#include "search2.hpp"
#include "sbpuzzle.hpp"

#ifndef __H
#define __H 3
#endif
#ifndef __W
#define __W 3
#endif

constexpr uint8_t H = __H, W = __W;

unsigned SEED = 42;

typedef sbpuzzle::SBPuzzleWHole<H, W> Puzzle;
typedef sbpuzzle::TileSwapAction Action;

std::queue<Puzzle> gpuzzles;

// specialize std::hash

template <class URNG>
void generate_puzzles(size_t n, std::queue<Puzzle> &puzzles, URNG &&rng) {
    typedef std::array<uint8_t, H*W> Tiles;
    Tiles tiles;
    std::iota(tiles.begin(), tiles.end(), 0);
    std::unordered_set<Puzzle> generated;
    size_t i = 0;
    while(n--) {
        i++;
        bool success = false;
        while(!success) {
            // first shuffle the puzzle and see if it is solvable
            std::shuffle(tiles.begin(), tiles.end(), rng);
            if(!sbpuzzle::tiles_solvable<H, W>(tiles)) continue;
            // if it is, check if it has already been generated before
            Puzzle p(tiles);
            if(generated.find(p) == generated.end()) { // if not, add it to the puzzles
                puzzles.emplace(p);
                success = true;
            }
        }
    }
}

int main(int argc, char *argv[]) {
    if(argc != 3) {
        std::cout << "Usage: ./generate_solutions num_puzzles out_file" << std::endl;
        return 0;
    }
    auto rng = std::default_random_engine(SEED);
    size_t n = std::stoull(argv[1]);
    generate_puzzles(n, gpuzzles, rng);
    while(!gpuzzles.empty()) {
        Puzzle p = gpuzzles.front(); gpuzzles.pop();
        auto r = search2::a_star_search<Puzzle, Action>(p, [](const Puzzle &p) { return p.manhattan_distance_to_solution(); });
    }
    return 0;
}
