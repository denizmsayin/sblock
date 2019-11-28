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

private:
    static const uint8_t X = 255;
    bool mask[SBPuzzle<H, W>::SIZE]; // true if in group, false if not
};

template <int H, int W>
MaskedSBPuzzle<H, W>::MaskedSBPuzzle(const bool imask[]) : SBPuzzle<H, W>() {
    std::copy(imask, imask + SBPuzzle<H, W>::SIZE, mask);
}

template <int H, int W>
MaskedSBPuzzle<H, W>::MaskedSBPuzzle(const int tiles[], const bool imask[]) 
    : SBPuzzle<H, W>(tiles) 
{
    std::copy(imask, imask + SBPuzzle<H, W>::SIZE, mask);
}

template <int H, int W>
MaskedSBPuzzle<H, W>::MaskedSBPuzzle(const SBPuzzle<H, W> &p, const bool imask[]) 
    : SBPuzzle<H, W>(p) 
{
    std::copy(imask, imask + SBPuzzle<H, W>::SIZE, mask);
}

template <int H, int W>
int MaskedSBPuzzle<H, W>::apply_action(Direction move) {
    int switch_pos = SBPuzzle<H, W>::get_switch_pos(move);
    int cost = mask[SBPuzzle<H, W>::tiles[switch_pos]] ? 1 : 0;
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
