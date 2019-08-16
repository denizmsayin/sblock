#ifndef __SBPUZZLE_H__
#define __SBPUZZLE_H__

#include <iostream>
#include <vector>

class SBPuzzle {
public:
    SBPuzzle(int h, int w);
    SBPuzzle(const std::vector<int> &tiles, int h, int w);

    /* Example output: 
     * -------------
     * | 0 | 1 | 2 |
     * -------------
     * | 3 | 4 | 5 |
     * -------------
     * | 6 | 7 | 8 |
     * -------------
     */
    friend std::ostream &operator<<(std::ostream &s, const SBPuzzle &p);

private:
    std::vector<int> tiles;
    int h, w;
};

#endif
