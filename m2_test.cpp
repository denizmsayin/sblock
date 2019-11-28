#include <iostream>

#include "sbpuzzle.hpp"

using namespace std;

int main() {
    bool mask[] = {true, true, true, true, false, false, false, false, false};
    SBPuzzle<3, 3> p(SBPuzzle<3, 3>::goal_state(), mask);
    cout << p << endl;
    for(auto a : p.possible_actions<MaskedAction>()) 
        cout << "(" << static_cast<int>(a.new_hole_pos) << ", " << static_cast<int>(a.new_tile_pos) << ") ";
    cout << endl;
    return 0;
}
