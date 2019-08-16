#include "sbpuzzle.h"

#include <iostream>
#include <vector>

using namespace std;
using Dir = SBPuzzle::Direction;

int main() {
    SBPuzzle puzzle(3, 3);
    cout << puzzle << std::endl;
    vector<Dir> v {Dir::LEFT, Dir::LEFT, Dir::UP};
    puzzle.apply_moves(v);
    cout << puzzle << std::endl;
    return 0;
}
