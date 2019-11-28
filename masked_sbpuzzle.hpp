#ifndef __MASKED_SBPUZZLE_HPP__
#define __MASKED_SBPUZZLE_HPP__

#include "sbpuzzle.hpp"

// a derivative class for generating disjoint pattern databases,
// where part of the puzzle remains indistinguishable
// the class is exactly the same apart from the apply_action function,
// since moving a tile which is not in the group is zero cost

// one significant issue for the disjoint pattern database implementation
// is that moves of the blank tile are not counted. This means that we would
// want to return a cost of 0 for moves where a tile from the group is not
// moved. Unfortunately, this renders BFS unoptimal due to not being a unit
// cost step, which hinders the construction of the database. Since it would
// be much more costly to run A* from each state to the puzzle solution,
// the only answer I could come up with is greatly extending the amount of
// actions possible so that each action contains all possible unit cost moves
// from the current state, which may imply multiple moves of the empty tile. 
// e.g. :
// ------------- Four possible unit cost moves:
// | 0 | 1 | X | ... 0 8 1 . 0 8 X . 0 1 X . 0 1 X
// ------------- ... 3 X X . 3 1 X . 8 3 X . 3 X X
// | 3 | X | 8 | ... 6 X X . 6 X X . 6 X X . 8 6 X
// -------------
// | 6 | X | X |
// -------------
// Another unfortunate consequence is that changing the signature of the possible
// actions function requires breaking inheritance and replacing it with composition.
// This is nice, but not very nice in the way that it requires forwarding of all functions

// A final observation of mine is that the classic SBPuzzle is a specific instance
// of the masked puzzle where all the tiles are in a single group. The only
// missing part now is that we want to be able to request using different sets of
// actions. To allow this, we simply need to make the possible_actions() member
// function templated, so that it can return different action types depending
// on the template argument type.

struct MaskedAction {
    uint8_t new_hole_pos, new_tile_pos;

    MaskedAction(uint8_t a, uint8_t b) : new_hole_pos(a), new_tile_pos(b) {}
};

template <int H, int W>
class MaskedSBPuzzle 
{
public:
    static const uint8_t _X = 255; // don't care tiles

    explicit MaskedSBPuzzle(const bool mask[]);
    explicit MaskedSBPuzzle(const int tiles[], const bool mask[]);
    explicit MaskedSBPuzzle(const SBPuzzle<H, W> &p, const bool mask[]);

    std::vector<MaskedAction> possible_actions() const;

    int apply_action(Direction move);
    int apply_action(MaskedAction a);

    bool is_solved() const;

    template <int HH, int WW>
    friend std::ostream &operator<<(std::ostream &s, const MaskedSBPuzzle<HH, WW> &p);

private:
    SBPuzzle<H, W> puzzle;
    void overwrite_tiles(const bool mask[]);
};

template <int H, int W>
void MaskedSBPuzzle<H, W>::overwrite_tiles(const bool mask[]) {
    auto &t = SBPuzzle<H, W>::tiles;
    for(int i = 0; i < SBPuzzle<H, W>::SIZE; ++i)
        if(t[i] != SBPuzzle<H, W>::HOLE && !mask[t[i]])
            t[i] = _X;
}

template <int H, int W>
MaskedSBPuzzle<H, W>::MaskedSBPuzzle(const bool imask[]) : SBPuzzle<H, W>() {
    overwrite_tiles(imask);
}

template <int H, int W>
MaskedSBPuzzle<H, W>::MaskedSBPuzzle(const int tiles[], const bool imask[]) 
    : SBPuzzle<H, W>(tiles) 
{
    overwrite_tiles(imask);
}

template <int H, int W>
MaskedSBPuzzle<H, W>::MaskedSBPuzzle(const SBPuzzle<H, W> &p, const bool imask[]) 
    : SBPuzzle<H, W>(p) 
{
    overwrite_tiles(imask);
}

template <int H, int W>
bool MaskedSBPuzzle<H, W>::is_solved() const {
    for(int i = 0; i < SBPuzzle<H, W>::SIZE; i++)
        if(SBPuzzle<H, W>::tiles[i] != _X && SBPuzzle<H, W>::tiles[i] != i)
            return false;
    return true;
}

template <int H, int W>
int MaskedSBPuzzle<H, W>::apply_action(Direction move) {
    int switch_pos = SBPuzzle<H, W>::get_switch_pos(move);
    int cost = SBPuzzle<H, W>::tiles[switch_pos] == _X ? 0 : 1;
    SBPuzzle<H, W>::move_tile(switch_pos);
    return cost;
}

template <int H, int W>
std::ostream &operator<<(std::ostream &s, const MaskedSBPuzzle<H, W> &p) {
    return stream_tiles(s, p.tiles, H, W, MaskedSBPuzzle<H, W>::_X);
}

// add hashability
namespace std {
    template <int H, int W>
    struct hash<MaskedSBPuzzle<H, W>> {
        size_t operator()(MaskedSBPuzzle<H, W> const &p) const noexcept {
            return p.hash();
        }
    };
}

#endif
