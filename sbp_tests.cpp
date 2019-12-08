#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <chrono>
#include <string>
#include <numeric>

#include "search2.hpp"

#include "sbpuzzle.hpp"
#include "pdb.hpp"
#include "dpdb.hpp"
#include "reflectdpdb.hpp"
#include "combineddb.hpp"
#include "crdpdb.hpp"

unsigned SEED = 42;
constexpr int N = 300;
constexpr int H = 3, W = 3;

// known given N, H, W & SEED
constexpr int OPTIMAL_MOVES = 6645;

//-------------------------------------------------------------------------------

using namespace std;
typedef sbpuzzle::SBPuzzleWHole<H, W> Puzzle;
typedef sbpuzzle::TileSwapAction Action;
typedef sbpuzzle::PDB<H, W> PDB;
typedef sbpuzzle::DPDB<H, W> DPDB;
typedef sbpuzzle::ReflectDPDB<H, W> ReflectDPDB;
typedef sbpuzzle::CombinedDB<H, W> CombinedDB;
typedef sbpuzzle::CRDPDB<H, W> CRDPDB;

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

class PDBHeuristic {
public:
    PDBHeuristic(const PDB *odb) : db(odb) {}

    int operator()(const Puzzle &p) {
        return db->lookup(p);
    }

private:
    const PDB *db;
};

template <typename F, typename... Args>
void test_function(const vector<Puzzle> &puzzles, F f, Args&&... args) {
    int num_moves = 0;
    for(size_t i = 0, size = puzzles.size(); i < size; ++i) {
        auto r = f(puzzles[i], args...);
        num_moves += r.cost;
        cout << '\r' << i << '/' << size << flush;
    }
    cout << '\r';
    if(num_moves == OPTIMAL_MOVES)
        cout << "Success!" << endl;
    else
        cout << "Failed. Total moves = " << num_moves << " instead of " << OPTIMAL_MOVES << endl;
}

void test_db(const std::vector<Puzzle> &puzzles, const PDB *db) {
        cout << "Testing A* with DB..." << endl;
        test_function(puzzles, search2::a_star_search<Puzzle, Action, PDBHeuristic>, PDBHeuristic(db));

        cout << "Testing IDA* with DB..." << endl;
        test_function(puzzles, search2::iterative_deepening_a_star<Puzzle, Action, PDBHeuristic>, PDBHeuristic(db));
}

int main() {
    cout << "**************************************" << endl;

    auto rng = default_random_engine(SEED);
    vector<Puzzle> puzzles;
    for(int i = 0; i < N; i++) 
        puzzles.push_back(create_solvable_puzzle<H, W>(rng));

    cout << "Testing BFS..." << endl;
    test_function(puzzles, search2::breadth_first_search<Puzzle, Action>);
    cout << "**************************************" << endl;

    cout << "Testing A* with Manhattan Distance..." << endl;
    test_function(puzzles, search2::a_star_search<Puzzle, Action, ManhattanHeuristic>, ManhattanHeuristic());
    cout << "**************************************" << endl;

    /*
    DPDB::generate_and_save(DBGROUPS, "tmpfile");
    cout << "Saved." << endl;
    DPDB db = DPDB::from_file("tmpfile");
    cout << "Read back successfully!" << endl;
    */

    for(const auto &group : DBGROUPS) {
        cout << "Generating sample DPDB & testing it..." << endl;
        for(auto x : group)
            std::cout << static_cast<int>(x) << " ";
        std::cout << std::endl;
        DPDB db = DPDB::generate(group);
        test_db(puzzles, &db);
        cout << "**************************************" << endl;
        cout << "Testing DPDB reflection..." << endl;
        ReflectDPDB rdb(db);
        test_db(puzzles, &rdb);
        cout << "**************************************" << endl;
        cout << "Testing combined DB & reflection..." << endl;
        std::vector<const PDB *> dbs {&db, &rdb};
        CombinedDB cdb(dbs);
        test_db(puzzles, &cdb);
        cout << "**************************************" << endl;
        cout << "Testing direct construction of CRDB..." << endl;
        CRDPDB cdb2 = CRDPDB::generate(group);
        test_db(puzzles, &cdb2);
        cout << "**************************************" << endl;
    }
    return 0;
}
