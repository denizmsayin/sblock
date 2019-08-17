#include "sbpuzzle.h"

#include <iostream>
#include <string>
#include <stdexcept>
#include <vector>
#include <list>
#include <queue>
#include <unordered_set>

SBPuzzle::SBPuzzle(int h, int w) : tiles(h*w), h(h), w(w), hole_pos(h*w-1)
{
    int size = h * w;
    for(int i = 0; i < size; i++)
        tiles[i] = i;
}

SBPuzzle::SBPuzzle(const std::vector<int> &tiles, int h, int w) : tiles(tiles), h(h), w(w) 
{
    hole_pos = find_hole();
}

bool SBPuzzle::is_solved() const {
    int size = h * w;
    for(int i = 0; i < size; i++)
        if(tiles[i] != i)
            return false;
    return true;
}

std::vector<SBPuzzle::Direction> SBPuzzle::possible_actions() const {
    std::vector<Direction> actions;
    if(hole_pos >= w) // not upper edge, can move UP
        actions.push_back(Direction::UP);
    if(hole_pos % w < w - 1) // not right edge, can move RIGHT
        actions.push_back(Direction::RIGHT);
    if(hole_pos < h*w - w) // not lower edge, can move DOWN
        actions.push_back(Direction::DOWN);
    if(hole_pos % w > 0) // not left edge, can move LEFT
        actions.push_back(Direction::LEFT);
    return actions;
}

int SBPuzzle::apply_action(Direction move) {
    int hole = h * w - 1;
    int switch_pos;
    switch(move) {
        case Direction::UP:    switch_pos = hole_pos - w; break;
        case Direction::RIGHT: switch_pos = hole_pos + 1; break;
        case Direction::DOWN:  switch_pos = hole_pos + w; break;
        case Direction::LEFT:  switch_pos = hole_pos - 1; break;
        default:               throw std::invalid_argument("Invalid move in move vector");
    }
    tiles[hole_pos] = tiles[switch_pos];
    tiles[switch_pos] = hole;
    hole_pos = switch_pos; // the new hole position
    return 1; // the path cost
}

bool SBPuzzle::operator==(const SBPuzzle &other) const {
    if(h == other.h && w == other.w) {
        int size = h * w;
        for(int i = 0; i < size; i++) {
            if(tiles[i] != other.tiles[i])
                return false;
        }
        return true;
    }
    return false;
}

std::ostream &operator<<(std::ostream &s, SBPuzzle::Direction dir) {
    using Dir = SBPuzzle::Direction;
    switch(dir) {
        case Dir::UP:       s << "U"; break;
        case Dir::RIGHT:    s << "R"; break;
        case Dir::LEFT:     s << "L"; break;
        case Dir::DOWN:     s << "D"; break;
        default:            s << "X"; break;
    }
    return s;
}

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

static int count_inversions(const std::vector<int> &v) {
    // count inversions with O(n^2) instead of mergesort style, 
    // since the vectors are very small
    int inversions = 0;
    int size = v.size();
    int hole = size - 1;
    for(int i = 0; i < size; i++) {
        if(v[i] != hole) {
            for(int j = i+1; j < size; j++) {
                if(v[i] > v[j]) {
                    inversions++;
                }
            }
        }
    }
    return inversions;
}

bool SBPuzzle::is_solvable() const {
    // see https://www.cs.bham.ac.uk/~mdr/teaching/modules04/java2/TilesSolvability.html
    int inversions = count_inversions(tiles);
    int hole_pos = find_hole();
    int hole_row_from_bottom = h - hole_pos % w;
    if(w % 2 == 1) // odd width
        return inversions % 2 == 0; // must have even inversions
    if(hole_row_from_bottom % 2 == 0) // even width
        return inversions % 2 == 1; // must have odd inversions
    // otherwise even
    return inversions % 2 == 1;
}

void SBPuzzle::apply_moves(const std::vector<Direction> &moves) {
    for(Direction d : moves) 
        apply_action(d);
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

size_t std::hash<SBPuzzle>::operator()(SBPuzzle const &p) const noexcept {
    return std::hash<std::string>()(p.encode());
}

std::string SBPuzzle::encode() const {
    // an important assumption here is that the puzzle has less than 256 tiles, which is a given
    // since we cannot solve puzzles larger than 5x5
    int size = h * w;
    std::string encoded;
    encoded.reserve(size);
    for(int i = 0; i < size; i++)
        encoded.push_back(tiles[i]);
    return encoded;
}
