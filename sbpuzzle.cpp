#include "sbpuzzle.h"

#include <string>

SBPuzzle::SBPuzzle(int h, int w) : h(h), w(w), tiles(h*w)
{
    int size = h * w;
    for(int i = 0; i < size; i++)
        tiles[i] = i;
}

SBPuzzle::SBPuzzle(const std::vector<int> &tiles, int h, int w) : tiles(tiles), h(h), w(w) {}

std::ostream &operator<<(std::ostream &s, const SBPuzzle &p) {
    int num_dashes = p.w * 4 + 1;
    std::string dash_str(num_dashes, '-');
    for(int i = 0, k = 0; i < p.h; i++) {
        s << dash_str << std::endl;
        for(int j = 0; j < p.w; j++)
            s << "| " << p.tiles[k++] << " ";
        s << "|" << std::endl;
    }
    s << dash_str;
    return s;
}
