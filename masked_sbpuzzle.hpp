#ifndef __MASKED_SBPUZZLE_HPP__
#define __MASKED_SBPUZZLE_HPP__

#include "sbpuzzle.hpp"

// a derivative class for generating disjoint pattern databases,
// where part of the puzzle remains indistinguishable
// the class is exactly the same apart from the apply_action function,
// since moving a tile which is not in the group is zero cost
template <int H, int W>
class MaskedSBPuzzle : public SBPuzzle<H, W> 
{
public:
    explicit MaskedSBPuzzle(const bool mask[]);
    explicit MaskedSBPuzzle(const int tiles[], const bool mask[]);
    explicit MaskedSBPuzzle(const SBPuzzle<H, W> &p, const bool mask[]);

    int apply_action(Direction move);
    bool is_solved() const;

private:
    static const uint8_t _X = 255;
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
