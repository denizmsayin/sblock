#include "sbpuzzle.hpp"

using namespace std;
using namespace sbpuzzle;

constexpr uint8_t H = 3, W = 3;

int main() {
    array<uint8_t, H*W> tiles {0, 1, 2, 3, 4, 5, 6, 7, 8};
    array<bool, H*W> mask {true, true, true, true, false, false, true, false, false};
    SBPuzzleNoHole<H, W> p(tiles, mask);
    cout << p << endl;
    for(auto a : p.action_generator<TileSwapAction>())
        cout << a << " ";
    cout << endl;
    return 0;
}
