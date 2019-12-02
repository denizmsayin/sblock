#include <iostream>

#include "sbpuzzle.hpp"
#include "search2.hpp"

using namespace std;
using sbpuzzle::SBPuzzle;

int main() {
    bool mask[] = {true, true, true, true, false, false, false, false, true};
    uint8_t tiles1[] = {0, 8, 2, 3, 1, 4, 5, 6, 7};
    SBPuzzle<3, 3> p(tiles1, mask);
    cout << p << endl;
    cout << p.goal_state() << endl;
    int cost = search2::breadth_first_search<SBPuzzle<3, 3>, sbpuzzle::TileSwapAction>(p);
    cout << cost << endl;
    return 0;
}
