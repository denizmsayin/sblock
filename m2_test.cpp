#include <iostream>

#include "sbpuzzle.hpp"
#include "search2.hpp"

using namespace std;
using sbpuzzle::SBPuzzle;

int main() {
    bool mask[] = {true, true, true, true, false, false, false, false, false};
    uint8_t tiles1[] = {0, 7, 2, 3, 1, 4, 5, 6, 8};
    SBPuzzle<3, 3> p(tiles1, mask);
    cout << p << endl;
    cout << p.goal_state() << endl;
    int cost = search2::breadth_first_search<SBPuzzle<3, 3>, sbpuzzle::TileSwapAction>(p);
    cout << cost << endl;
    for(auto a : p.possible_actions<sbpuzzle::TileSwapAction>()) 
        cout << '(' << ((int) a.tpos) << ", " << ((int) a.hpos) << ") ";
    cout << endl;
    return 0;
}
