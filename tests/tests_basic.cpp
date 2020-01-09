#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <chrono>
#include <string>
#include <numeric>

#include "../search.hpp"
#include "../sbpuzzle.hpp"

unsigned SEED = 42;
constexpr int N = 300;
constexpr int H = 3, W = 3;

// known given N, H, W & SEED
constexpr int OPTIMAL_MOVES = 6645;

//-------------------------------------------------------------------------------

namespace search2 = denizmsayin::sblock::search;
namespace sbp = denizmsayin::sblock::sbpuzzle;
using namespace std;

typedef sbp::Basic<H, W> Puzzle;
typedef sbp::TileSwapAction Action;
typedef sbp::heuristics::pdb::Base<H, W> PDB;
typedef sbp::heuristics::pdb::DPDB<H, W> DPDB;
typedef sbp::heuristics::pdb::ReflectDPDBDependent<H, W> ReflectDPDB;
typedef sbp::heuristics::pdb::CombinedDB<H, W> CombinedDB;
typedef sbp::heuristics::pdb::CRDPDB<H, W> CRDPDB;

std::vector<array<uint8_t, H*W>> DBGROUPS { 
    {0, 0, 0, 1, 1, 1, 2, 2, sbp::DONT_CARE},
    {0, 0, 0, 1, 1, 0, 1, 1, sbp::DONT_CARE} 
};
// std::vector<const char *> DBFILES {"a1.db", "a2.db", "a3.db"};

template <int H, int W, class URNG>
Puzzle create_solvable_puzzle(URNG &&rng) {
    std::array<uint8_t, H*W> tiles;
    std::iota(tiles.begin(), tiles.end(), 0);
    do {
        std::shuffle(tiles.begin(), tiles.end(), rng); 
    } while(!sbp::tiles_solvable<H, W>(tiles));
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

template <typename F>
void test_function(const vector<Puzzle> &puzzles, F f) {
    int num_moves = 0;
    for(size_t i = 0, size = puzzles.size(); i < size; ++i) {
        auto r = f(puzzles[i]);
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
        PDBHeuristic dbh(db);

        cout << "Testing A* with DB..." << endl;
        test_function(puzzles, [&](const Puzzle &p) { 
                return search2::a_star_search<Puzzle, Action>(p, dbh); 
        });

        cout << "Testing IDA* with DB..." << endl;
        test_function(puzzles, [&](const Puzzle &p) {
                return search2::iterative_deepening_a_star<Puzzle, Action>(p, dbh);
        });
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
    test_function(puzzles, [](const Puzzle &p) {
            return search2::a_star_search<Puzzle, Action, ManhattanHeuristic>(p);
    });
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
