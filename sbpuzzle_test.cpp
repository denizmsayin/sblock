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

unsigned SEED = 42;

constexpr int N = 300;
constexpr int H = 4, W = 4;
const std::string dbfile = "dp4x4.db";

#define USEDB

//-------------------------------------------------------------------------------

#ifdef USEDB
#include "dpdb.hpp"
using sbpuzzle::DPDB;
//uint8_t DBGROUPS[] = {1, 1, 1, 0, 0, 1, 0, 0, sbpuzzle::DONT_CARE};
//std::vector<const char *> DBFILES {"g1.db", "g2.db"};
// uint8_t DBGROUPS[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
// std::vector<const char *> DBFILES {"gmax.db"};
// uint8_t DBGROUPS[] = {0, 1, 2, 3, 4, 5, 6, 7, 0};
// std::vector<const char *> DBFILES {"m0.db", "m1.db", "m2.db", "m3.db", "m4.db", "m5.db", "m6.db", "m7.db"};
DPDB<H, W> DB = DPDB<H, W>::from_file(dbfile);
#endif

using namespace std;
using TSA = sbpuzzle::TileSwapAction;
typedef sbpuzzle::SBPuzzleWHole<H, W> Puzzle;

template <int H, int W, class URNG>
Puzzle create_solvable_puzzle(URNG &&rng) {
    std::array<uint8_t, H*W> tiles;
    std::iota(tiles.begin(), tiles.end(), 0);
    do {
        std::shuffle(tiles.begin(), tiles.end(), rng); 
    } while(!sbpuzzle::tiles_solvable<H, W>(tiles));
    return Puzzle(tiles);
}

/*
template <int H, int W, class URNG>
SBPuzzle<H, W> create_solvable_masked_puzzle(URNG &&rng) {
    SBPuzzle<H, W> puzzle;
    bool mask[H*W];
    for(int i = 0, size = H*W; i < size; ++i)
        mask[i] = true;
    mask[H*W-1] = false;
    do {
        puzzle.shuffle(rng);
    } while(!puzzle.is_solvable());
    return SBPuzzle<H, W>(puzzle, mask);
}
*/

class ZeroHeuristic {
public:
    constexpr uint64_t operator()(const Puzzle &p) {
        return 0;
    }
};

/*
template <int H, int W>
class MisplacedTilesHeuristic {
public:
    int operator()(const SBPuzzle<H, W> &p) {
        return p.num_misplaced_tiles();
    }
};
*/

class ManhattanHeuristic {
public:
    uint64_t operator()(const Puzzle &p) {
        return p.manhattan_distance_to_solution();
    }
};

#ifdef USEDB
class DPDBHeuristic {
public:
    int operator()(const Puzzle &p) {
        return p.lookup_cost(DB);
    }
};
#endif

/*
bool check_equality(const vector<Dir> &m1, const vector<Dir> &m2) {
    bool eq = false;
    if(m1.size() == m2.size()) {
        int size = m1.size();
        eq = true;
        for(int i = 0; i < size; i++)
            if(m1[i] != m2[i]) {
                eq = false;
                break;
            }
    }
    return eq;
}
*/

int main() {
    auto rng = default_random_engine(SEED);
    vector<Puzzle> puzzles;
    for(int i = 0; i < N; i++) 
        puzzles.push_back(create_solvable_puzzle<H, W>(rng));

    auto t1 = chrono::high_resolution_clock::now();
    int num_moves = 0;
    size_t num_nodes = 0;
    for(int i = 0; i < N; i++) {
        /*
        int m1 = search2::a_star_search<SBPuzzle<H, W>, TSA, 
                                        ManhattanHeuristic<H, W>>(puzzles[i]); 
        #ifdef USEDB
        int m2 = search2::a_star_search<SBPuzzle<H, W>, TSA, 
                                        DPDBHeuristic<H, W>>(puzzles[i]); 
        #endif
        if(m1 != m2) {
            cout << "DB Result: " << m2 << ", MD Result: " << m1 << endl;
            cout << puzzles[i] << endl;
            cout << "**********************" << endl;
        }
        */
        // num_moves += search2::breadth_first_search<Puzzle, TSA>(puzzles[i]);
        // std::cout << puzzles[i] << std::endl;
        int m = search2::a_star_search<Puzzle, TSA, DPDBHeuristic>(puzzles[i]);
        // std::cout << "Solved in " << m << " moves." << std::endl;
        num_moves += m;
        // num_moves += search2::iterative_deepening_a_star<Puzzle, TSA, DPDBHeuristic>(puzzles[i]);
        // num_moves += search2::a_star_search<Puzzle, TSA, ManhattanHeuristic>(puzzles[i]);
        // num_moves += moves.size();
        num_nodes += search2::get_node_counter<Puzzle>();
        // search2::reset_node_counter<SBPuzzle<H, W>>();
        cout << '\r' << (i+1) << "/" << N << flush;
    }
    cout << '\r';
    auto t2 = chrono::high_resolution_clock::now();

    chrono::duration<double, std::milli> fp_ms = t2 - t1;

    double avg_moves = static_cast<double>(num_moves) / N;
    double avg_nodes = static_cast<double>(num_nodes) / N;
    cout << "Solved " << N << " puzzles in " << fp_ms.count() / N << " milliseconds on "
         << "average with " << avg_moves << " moves and " << avg_nodes 
         << " nodes generated on average." << endl;

    return 0;
}
