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
constexpr int H = 3, W = 3;

#define USEDB

//-------------------------------------------------------------------------------

#ifdef USEDB
#include "dpdb.hpp"
using sbpuzzle::DPDB;

//uint8_t DBGROUPS[] = {1, 1, 1, 0, 0, 1, 0, 0, sbpuzzle::DONT_CARE};
//std::vector<const char *> DBFILES {"g1.db", "g2.db"};
std::array<uint8_t, H*W> DBGROUPS {0, 0, 0, 1, 1, 1, 2, 2, sbpuzzle::DONT_CARE};
std::vector<const char *> DBFILES {"a1.db", "a2.db", "a3.db"};
// uint8_t DBGROUPS[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
// std::vector<const char *> DBFILES {"gmax.db"};
// uint8_t DBGROUPS[] = {0, 1, 2, 3, 4, 5, 6, 7, 0};
// std::vector<const char *> DBFILES {"m0.db", "m1.db", "m2.db", "m3.db", "m4.db", "m5.db", "m6.db", "m7.db"};
DPDB<H, W> DB(DBGROUPS, DBFILES.begin(), DBFILES.end());
#endif

using namespace std;
using sbpuzzle::SBPuzzle;
using TSA = sbpuzzle::TileSwapAction;

template <int H, int W, class URNG>
SBPuzzle<H, W> create_solvable_puzzle(URNG &&rng) {
    std::array<uint8_t, H*W> tiles;
    std::iota(tiles.begin(), tiles.end(), 0);
    do {
        std::shuffle(tiles.begin(), tiles.end(), rng); 
    } while(!sbpuzzle::tiles_solvable<H, W>(tiles));
    return SBPuzzle<H, W>(tiles);
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

template <int H, int W>
class ZeroHeuristic {
public:
    constexpr uint64_t operator()(const SBPuzzle<H, W> &p) {
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

template <int H, int W>
class ManhattanHeuristic {
public:
    uint64_t operator()(const SBPuzzle<H, W> &p) {
        return p.manhattan_distance_to_solution();
    }
};

#ifdef USEDB
template <int H, int W>
class DPDBHeuristic {
public:
    int operator()(const SBPuzzle<H, W> &p) {
        return DB.lookup(p);
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
    vector<SBPuzzle<H, W>> puzzles;
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
        // num_moves += search2::breadth_first_search<SBPuzzle<H, W>, TSA>(puzzles[i]);
        // num_moves += search2::a_star_search<SBPuzzle<H, W>, TSA, DPDBHeuristic<H, W>>(puzzles[i]);
        num_moves += search2::a_star_search<SBPuzzle<H, W>, TSA, ManhattanHeuristic<H, W>>(puzzles[i]);
        // num_moves += moves.size();
        num_nodes += search2::get_node_counter<SBPuzzle<H, W>>();
        // search2::reset_node_counter<SBPuzzle<H, W>>();
        cout << '\r' << i << "/" << N << flush;
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
