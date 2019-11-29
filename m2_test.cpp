#include <iostream>

#include "sbpuzzle.hpp"

using namespace std;

template <int H, int W>
void prop_test(const SBPuzzle<H, W> &p) {
    cout << "Propagating:" << endl;
    p.propagate_hole();
    cout << p << endl;
    cout << "Unpropagating:" << endl;
    p.unpropagate_hole();
    cout << p << endl;
}

int main() {
    bool mask[] = {true, true, true, true, false, false, false, false, false};
    uint8_t tiles1[] = {0, 8, 2, 3, 1, 4, 5, 6, 7};
    SBPuzzle<3, 3> p(tiles1, mask);
    cout << p << endl;
    prop_test(p);
    uint8_t tiles[] = {0, 7, 2, 3, 1, 4, 5, 6, 8};
    SBPuzzle<3, 3> p2(tiles, mask);
    cout << p2 << endl;
    prop_test(p2);
    cout << (p == p2) << endl;
    cout << p.hash() << " " << p2.hash() << endl;
    return 0;
}
