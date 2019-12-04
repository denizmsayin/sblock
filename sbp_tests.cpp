#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <chrono>
#include <string>
#include <numeric>

#include "search.hpp"
#include "search2.hpp"
#include "search_queues.hpp"
#include "sbpuzzle.hpp"
#include "dpdb.hpp"

unsigned SEED = 42;
constexpr int N = 300;
constexpr int H = 3, W = 3;

// known given N, H, W & SEED
constexpr int OPTIMAL_MOVES = 6645;

//-------------------------------------------------------------------------------

using namespace std;
typedef sbpuzzle::SBPuzzle<H, W> Puzzle;
typedef sbpuzzle::TileSwapAction Action;
typedef sbpuzzle::DPDB<H, W> DPDB;

uint8_t DBGROUPS[] = {0, 0, 0, 1, 1, 1, 2, 2, sbpuzzle::DONT_CARE};
std::vector<const char *> DBFILES {"a1.db", "a2.db", "a3.db"};
DPDB DB(DBGROUPS, DBGROUPS + 9, DBFILES.begin(), DBFILES.end());

template <int H, int W, class URNG>
Puzzle create_solvable_puzzle(URNG &&rng) {
    constexpr auto size = H*W;
    uint8_t tiles[H*W];
    std::iota(tiles, tiles + size, 0);
    do {
        std::shuffle(tiles, tiles + size, rng); 
    } while(!sbpuzzle::tiles_solvable<H, W>(tiles));
    return Puzzle(tiles);
}

class ManhattanHeuristic {
public:
    uint64_t operator()(const Puzzle &p) {
        return p.manhattan_distance_to_solution();
    }
};

class DPDBHeuristic {
public:
    int operator()(const Puzzle &p) {
        return DB.lookup(p);
    }
};

template <typename F>
void test_function(const vector<Puzzle> &puzzles, F f) {
    int num_moves = 0;
    for(size_t i = 0, size = puzzles.size(); i < size; ++i) {
        num_moves += f(puzzles[i]);
        cout << '\r' << i << '/' << size << flush;
    }
    cout << '\r';
    if(num_moves == OPTIMAL_MOVES)
        cout << "Success!" << endl;
    else
        cout << "Failed. Total moves = " << num_moves << " instead of " << OPTIMAL_MOVES << endl;
}

int main() {
    auto rng = default_random_engine(SEED);
    vector<Puzzle> puzzles;
    for(int i = 0; i < N; i++) 
        puzzles.push_back(create_solvable_puzzle<H, W>(rng));

    cout << "Testing BFS..." << endl;
    test_function(puzzles, search2::breadth_first_search<Puzzle, Action>);

    cout << "Testing A* with Manhattan Distance..." << endl;
    test_function(puzzles, search2::a_star_search<Puzzle, Action, ManhattanHeuristic>);

    cout << "Testing A* with DPDB..." << endl;
    test_function(puzzles, search2::a_star_search<Puzzle, Action, DPDBHeuristic>);

    cout << "Testing IDA* with DPDB..." << endl;
    test_function(puzzles, search2::iterative_deepening_a_star<Puzzle, Action, DPDBHeuristic>);

    return 0;
}
