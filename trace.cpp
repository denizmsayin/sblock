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

constexpr int H = 3, W = 3;

//-------------------------------------------------------------------------------

#include "dpdb.hpp"
using sbpuzzle::DPDB;

uint8_t DBGROUPS[] = {1, 1, 1, 0, 0, 1, 0, 0, sbpuzzle::DONT_CARE};
std::vector<const char *> DBFILES {"g1.db", "g2.db"};
// uint8_t DBGROUPS[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
// std::vector<const char *> DBFILES {"gmax.db"};
// uint8_t DBGROUPS[] = {0, 1, 2, 3, 4, 5, 6, 7, 0};
// std::vector<const char *> DBFILES {"m0.db", "m1.db", "m2.db", "m3.db", "m4.db", "m5.db", "m6.db", "m7.db"};
DPDB<H, W> DB(DBGROUPS, DBGROUPS + 9, DBFILES.begin(), DBFILES.end());

using namespace std;
using sbpuzzle::SBPuzzle;
using TSA = sbpuzzle::TileSwapAction;

template <int H, int W>
class ManhattanHeuristic {
public:
    uint64_t operator()(const SBPuzzle<H, W> &p) {
        return p.manhattan_distance_to_solution();
    }
};

template <int H, int W>
class DPDBHeuristic {
public:
    int operator()(const SBPuzzle<H, W> &p) {
        return DB.lookup(p);
    }
};

int main() {
    using TSA = sbpuzzle::TileSwapAction;
    const uint8_t tiles[] = {5, 8, 3, 7, 0, 6, 4, 1, 2};
    SBPuzzle<H, W> s(tiles);
    search2::BreadthFirstIterator<SBPuzzle<H, W>, TSA> itr(s);
    size_t i = 0;
    SeriesTracker<size_t> t(&i);
    std::cout << search2::a_star_search<SBPuzzle<H, W>, TSA, DPDBHeuristic<H,W>>(s) << endl;
    std::cout << search2::a_star_search<SBPuzzle<H, W>, TSA, ManhattanHeuristic<H,W>>(s) << endl;
    while(!itr.done()) {
        auto node = itr.next();
        cout << node.puzzle << endl;
        auto db_est = DPDBHeuristic<H, W>()(node.puzzle);
        auto man_est = ManhattanHeuristic<H, W>()(node.puzzle);
        auto sol = search2::a_star_search<SBPuzzle<H, W>, TSA, 
                                          ManhattanHeuristic<H, W>>(node.puzzle);
        if(db_est >= sol) {
            cout << node.puzzle << endl;
            cout << "True cost: " << sol << endl;
            cout << "Manh. D. est: " << man_est << endl;
            cout << "DPDB est.: " << db_est << endl;
        }

        i++;
        t.track();
    }
    return 0;
}
