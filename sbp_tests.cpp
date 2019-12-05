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

std::vector<array<uint8_t, H*W>> DBGROUPS { 
    {0, 0, 0, 1, 1, 1, 2, 2, sbpuzzle::DONT_CARE},
    {0, 0, 0, 1, 1, 0, 1, 1, sbpuzzle::DONT_CARE} 
};
// std::vector<const char *> DBFILES {"a1.db", "a2.db", "a3.db"};

template <int H, int W, class URNG>
Puzzle create_solvable_puzzle(URNG &&rng) {
    std::array<uint8_t, H*W> tiles;
    std::iota(tiles.begin(), tiles.end(), 0);
    do {
        std::shuffle(tiles.begin(), tiles.end(), rng); 
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
    DPDBHeuristic(const DPDB &odb) : db(odb) {}

    int operator()(const Puzzle &p) {
        return p.lookup_cost(db);
    }

private:
    const DPDB &db;
};

template <typename F, typename... Args>
void test_function(const vector<Puzzle> &puzzles, F f, Args&&... args) {
    int num_moves = 0;
    for(size_t i = 0, size = puzzles.size(); i < size; ++i) {
        num_moves += f(puzzles[i], args...);
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

    cout << "Testing DQ-BFS..." << endl;
    test_function(puzzles, search2::bfs_double_q<Puzzle, Action>);

    cout << "Testing A* with Manhattan Distance..." << endl;
    test_function(puzzles, search2::a_star_search<Puzzle, Action, ManhattanHeuristic>, ManhattanHeuristic());

    /*
    DPDB::generate_and_save(DBGROUPS, "tmpfile");
    cout << "Saved." << endl;
    DPDB db = DPDB::from_file("tmpfile");
    cout << "Read back successfully!" << endl;
    */

    for(const auto &group : DBGROUPS) {
        cout << "Generating sample DPDB... ";
        for(auto x : group)
            std::cout << static_cast<int>(x) << " ";
        std::cout << std::endl;
        DPDB db = DPDB::generate(group);
        cout << "Testing A* with DPDB..." << endl;
        test_function(puzzles, search2::a_star_search<Puzzle, Action, DPDBHeuristic>, DPDBHeuristic(db));

        cout << "Testing IDA* with DPDB..." << endl;
        test_function(puzzles, search2::iterative_deepening_a_star<Puzzle, Action, DPDBHeuristic>, DPDBHeuristic(db));
    }
    return 0;
}
