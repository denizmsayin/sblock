#include "sbpuzzle.h"

#include <string>
#include <stdexcept>

SBPuzzle::SBPuzzle(int h, int w) : tiles(h*w), h(h), w(w)
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

void SBPuzzle::apply_moves(const std::vector<Direction> moves) {
    int hole = h * w - 1;
    int hole_pos = find_hole();
    for(Direction d : moves) {
        int switch_pos;
        switch(d) {
            case Direction::UP:    switch_pos = hole_pos - w; break;
            case Direction::RIGHT: switch_pos = hole_pos + 1; break;
            case Direction::DOWN:  switch_pos = hole_pos + w; break;
            case Direction::LEFT:  switch_pos = hole_pos - 1; break;
        }
        tiles[hole_pos] = tiles[switch_pos];
        tiles[switch_pos] = hole;
        hole_pos = switch_pos;
    }
}

int SBPuzzle::find_hole() const {
    int size = h * w;
    int hole = size - 1;
    for(int i = 0; i < size; i++)
        if(tiles[i] == hole)
            return i;
    throw std::invalid_argument("Puzzle does not contain a hole");
    return 0;
}
