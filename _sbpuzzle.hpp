#ifndef __SBPUZZLE_H__
#define __SBPUZZLE_H__

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <cstdint>

#include "sblock_utils.hpp"

// forward declare disjoint pattern db for friend-classing
template <int H, int W>
class DPDB;

// NOTE: the derivative masked class has been included inside the main SBPuzzle class
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
// Another consequence of this application is that since all positions of the hole are
// equivalent, they should be undistinguishable state-wise. This means that a slight
// change is necessary for equality checking and hashing.

// A final observation of mine is that the classic SBPuzzle is a specific instance
// of the masked puzzle where all the tiles are in a single group. The only
// missing part now is that we want to be able to request using different sets of
// actions. To allow this, we simply need to make the possible_actions() member
// function templated, so that it can return different action types depending
// on the template argument type.

template <int H, int W>
class SBPuzzle {
public:
    
    // an enumeration for specifying actions on the puzzle
    enum class Direction : uint8_t {
        UP,
        RIGHT,
        DOWN,
        LEFT,
        INVALID
    };

    // a more complicated action class for masked puzzles
    struct ExpandedAction {
        uint8_t new_hole_pos, new_tile_pos;

        ExpandedAction(uint8_t a, uint8_t b) : new_hole_pos(a), new_tile_pos(b) {}
    };

    // Initializes an hxw sliding block puzzle with a solved configuration
    explicit SBPuzzle();
    // Initializes an hxw sliding block puzzle with the provided configuration
    explicit SBPuzzle(const uint8_t tiles[]);
    template <class Iterator>
    explicit SBPuzzle(Iterator begin, Iterator end);

    // for initializing with a mask
    explicit SBPuzzle(const bool mask[]);
    explicit SBPuzzle(const uint8_t tiles[], const bool mask[]);
    explicit SBPuzzle(const SBPuzzle<H, W> &p, const bool mask[]);

    SBPuzzle(const SBPuzzle &other) = default;
    SBPuzzle(SBPuzzle &&other) = default;
    SBPuzzle &operator=(const SBPuzzle &other) = default;
    SBPuzzle &operator=(SBPuzzle &&other) = default;

    // return the goal state for this puzzle
    static SBPuzzle goal_state();

    // returns True if the puzzle is in a solved state
    bool is_solved() const;

    // returns the possible actions on the puzzle
    template <class Action>
    std::vector<Action> possible_actions() const;


    // apply the move in the given direction using the cached hole position
    // return the path cost, which is 1 for our case for any choice
    int apply_action(Direction move);
    int apply_action(ExpandedAction ma);

    // return true if the puzzle is part of the solvable permutations
    bool is_solvable() const;

    bool operator==(const SBPuzzle &other) const; // equality check for hashing

    // shuffle the puzzle randomly
    template <class URNG>
    void shuffle(URNG &&urng) {
        std::shuffle(tiles, tiles + SIZE, urng);
        hole_pos = find_hole();
    }

    // Applies the list of moves on the puzzle. The direction specifies which side of
    // the hole is to be moved. e.g.:
    // -------------
    // | 0 | 2 | 1 |
    // -------------
    // | 3 | 8 | 5 |
    // -------------
    // | 6 | 7 | 4 |
    // -------------
    // In the state above, knowing that 8 is the hole, 'UP' would mean that 2 and 8 are
    // swapped
    template <typename Iterator>
    void apply_moves(Iterator begin, Iterator end);

    // Operator overload for output streams. Example output for a solved 3x3 puzzle:
    // -------------
    // | 0 | 1 | 2 |
    // -------------
    // | 3 | 4 | 5 |
    // -------------
    // | 6 | 7 | 8 |
    // -------------
    template <int HH, int WW>
    friend std::ostream &operator<<(std::ostream &s, const SBPuzzle<HH, WW> &p);

    size_t hash() const;

    // heuristic for misplaced tiles
    int num_misplaced_tiles() const;

    // heuristic for manhattan distance
    int manhattan_distance_to_solution() const;

    friend class DPDB<H, W>;

private:
public:
    static const uint8_t _X = 255;

    uint8_t tiles[H*W];
    uint8_t hole_pos; // cached
    bool hole_masked; // hole is in the given group or not?

    int find_hole() const;

    constexpr static int SIZE = H * W;
    constexpr static int HOLE = H * W - 1;
    static int OFFSETS[]; // UP, RIGHT, DOWN, LEFT

    int get_switch_pos(Direction move) const;
    void move_tile(int switch_pos);
    void overwrite_tiles(const bool mask[]);
    void propagate_hole(uint8_t hole_pos);
    void propagate_hole();
    void unpropagate_hole();

    void mark_valid_moves(bool directions[]) const;
    void add_expanded_actions(int pos, std::vector<ExpandedAction> &v);

    // hacky helpers for equality & hashing when the hole is not in the group
    void propagate_hole() const;
    void unpropagate_hole() const;

    // since it is not possible to specialize a templated function of an unspecialized
    // templated class, in our case the possible_actions() function, we need a dummy
    // templated wrapper struct to which we can delegate possible_actions(). However,
    // it is not possible to FULLY specialize the wrapper struct either, only partially.
    // Which is why we need a struct with the actual template parameter and a dummy one.
    // C++ hell, anyone?! Wow, templates are C++'s strength but also quite messed up.
    template <typename Action, typename Dummy = void>
    struct PADelegate {
        static std::vector<Action> f(const SBPuzzle &p);
    };
};

template <int H, int W>
int SBPuzzle<H, W>::OFFSETS[] = {-W, +1, +W, -1};

template <int H, int W>
static void _mark_valid_moves(int p, bool directions[]) {
    static constexpr int SIZE = H*W;
    int rem = p % W;
    directions[0] = p >= W; // up
    directions[1] = rem < W-1; // right
    directions[2] = p < SIZE-W; // down
    directions[3] = rem > 0; // left
}

template <int H, int W>
void SBPuzzle<H, W>::mark_valid_moves(bool directions[]) const {
    _mark_valid_moves<H, W>(hole_pos, directions);
}

template <int H, int W>
template <typename Dummy>
struct SBPuzzle<H, W>::PADelegate<typename SBPuzzle<H, W>::Direction, Dummy> {
    static auto f(const SBPuzzle<H, W> &p) {
        std::vector<SBPuzzle<H, W>::Direction> actions;
        bool conds[4];
        p.mark_valid_moves(conds);
        for(int i = 0; i < 4; i++)
            if(conds[i])
                actions.emplace_back(static_cast<SBPuzzle<H, W>::Direction>(i));
        return actions;
    }
};

// a helper function for the masked action generator, kind of a simplified dfs
// note that this leaves the puzzle in an invalid state for further use and
// is only intended as a helper private function
template <int H, int W>
void SBPuzzle<H, W>::propagate_hole(uint8_t hp) {
    bool conds[4];
    _mark_valid_moves<H, W>(hp, conds);
    for(int i = 0; i < 4; ++i) {
        int nhp = hp + OFFSETS[i];
        if(conds[i] && tiles[nhp] == _X) {
            tiles[nhp] = HOLE;
            propagate_hole(nhp);
        }
    }
}

template <int H, int W>
void SBPuzzle<H, W>::propagate_hole() {
    propagate_hole(hole_pos);
}

template <int H, int W>
void SBPuzzle<H, W>::unpropagate_hole() {
    for(auto i = 0; i < SIZE; ++i)
        if(tiles[i] == HOLE)
            tiles[i] = _X;
    tiles[hole_pos] = HOLE;
}


template <int H, int W>
void SBPuzzle<H, W>::add_expanded_actions(int i, std::vector<SBPuzzle<H, W>::ExpandedAction> &v) 
{
    // add possible actions around a position to the vector v
    // loop through possible directions
    bool conds[4];
    _mark_valid_moves<H, W>(i, conds);
    for(auto j = 0; j < 4; ++j) {
        if(conds[j]) {
            // if the position is not a hole or X, add it to actions
            auto pos = i + SBPuzzle<H, W>::OFFSETS[j];
            if(tiles[pos] != _X && tiles[pos] != HOLE) 
            {
                v.emplace_back(pos, i);
            }
        }
    }
}

template <int H, int W>
template <typename Dummy>
struct SBPuzzle<H, W>::PADelegate<typename SBPuzzle<H, W>::ExpandedAction, Dummy> {
    static auto f(const SBPuzzle<H, W> &puzzle) {
        std::vector<SBPuzzle<H, W>::ExpandedAction> actions;
        // explore X states around the hole, all tiles reachable from those are a possibility
        // create a dummy copy puzzle
        auto cp = puzzle;
        if(cp.hole_masked) { // hole is in the group, no need to propagate, EA is useless
            cp.add_expanded_actions(cp.hole_pos, actions);
        } else {
            cp.propagate_hole();
            // now, go over tiles reachable from holes and add them as actions
            for(auto i = 0; i < SBPuzzle<H, W>::SIZE; ++i) 
                if(cp.tiles[i] == SBPuzzle<H, W>::HOLE) 
                    cp.add_expanded_actions(i, actions);
        }
        return actions;
    }
};

template <int H, int W>
void SBPuzzle<H, W>::overwrite_tiles(const bool mask[]) {
    for(int i = 0; i < SIZE; ++i)
        if(tiles[i] != HOLE && !mask[tiles[i]])
            tiles[i] = _X;
    hole_masked = mask[HOLE];
}

template <int H, int W>
SBPuzzle<H, W>::SBPuzzle(const bool imask[]) : SBPuzzle<H, W>() {
    overwrite_tiles(imask);
}

template <int H, int W>
SBPuzzle<H, W>::SBPuzzle(const uint8_t tiles[], const bool imask[]) : SBPuzzle<H, W>(tiles) 
{
    overwrite_tiles(imask);
}

template <int H, int W>
SBPuzzle<H, W>::SBPuzzle(const SBPuzzle<H, W> &p, const bool imask[]) : SBPuzzle<H, W>(p) 
{
    overwrite_tiles(imask);
}

template <int H, int W>
std::ostream &operator<<(std::ostream &s, const SBPuzzle<H, W> &p) {
    const uint8_t *max = std::max_element(p.tiles, p.tiles + SBPuzzle<H, W>::SIZE);
    uint8_t num_digits = (*max > 99) ? 3 : ((*max > 9) ? 2 : 1); 
    int num_dashes = W * (num_digits + 3) + 1;
    std::string dash_str(num_dashes, '-');
    std::string empty_str(num_digits - 1, ' ');
    for(size_t i = 0, k = 0; i < H; i++) {
        s << dash_str << std::endl;
        for(size_t j = 0; j < W; j++) {
            s << "| "; 
            if(p.tiles[k] == SBPuzzle<H, W>::_X) // marks masked tiles, big dep issue!
                s << empty_str << "X ";
            else 
                s << std::setw(num_digits) << static_cast<int>(p.tiles[k]) << " ";
            ++k;
        }
        s << "|" << std::endl;
    }
    s << dash_str;
    return s;
}

template <int H, int W>
typename SBPuzzle<H, W>::Direction inverse(typename SBPuzzle<H, W>::Direction d) {
    using Dir = typename SBPuzzle<H, W>::Direction;
    switch(d) {
        case Dir::UP:       return Dir::DOWN;
        case Dir::RIGHT:    return Dir::LEFT;
        case Dir::DOWN:     return Dir::UP;
        case Dir::LEFT:     return Dir::RIGHT;
        default:            return Dir::INVALID;
    }
}

template <int H, int W>
SBPuzzle<H, W>::SBPuzzle() : hole_pos(HOLE), hole_masked(true)
{
    for(int i = 0; i < SIZE; i++)
        tiles[i] = i;
}

template <int H, int W>
SBPuzzle<H, W>::SBPuzzle(const uint8_t o_tiles[]) : hole_masked(true)
{
    for(int i = 0; i < SIZE; i++) {
        if(o_tiles[i] == HOLE)
            hole_pos = i;
        tiles[i] = o_tiles[i];
    }
}

template <int H, int W>
template <typename Iterator>
SBPuzzle<H, W>::SBPuzzle(Iterator begin, Iterator end) : hole_masked(true)
{
    std::copy(begin, end, tiles);
}

template <int H, int W>
SBPuzzle<H, W> SBPuzzle<H, W>::goal_state() {
    return SBPuzzle<H, W>();
}

template <int H, int W>
bool SBPuzzle<H, W>::is_solved() const {
    for(int i = 0; i < SBPuzzle<H, W>::SIZE; i++)
        if(tiles[i] != _X && tiles[i] != i) {
            if(tiles[i] != HOLE)
                return false;
            else if(hole_masked)
                return false;
        }
    return true;
}

template <int H, int W>
template <typename Action>
std::vector<Action> SBPuzzle<H, W>::possible_actions() const {
    return PADelegate<Action>::f(*this);
}

template <int H, int W>
int SBPuzzle<H, W>::get_switch_pos(Direction move) const {
    return hole_pos + OFFSETS[static_cast<int>(move)];
}

template <int H, int W>
void SBPuzzle<H, W>::move_tile(int switch_pos) {
    tiles[hole_pos] = tiles[switch_pos];
    tiles[switch_pos] = HOLE;
    hole_pos = switch_pos; // the new hole position
}

template <int H, int W>
int SBPuzzle<H, W>::apply_action(Direction move) {
    int switch_pos = SBPuzzle<H, W>::get_switch_pos(move);
    int cost = (SBPuzzle<H, W>::tiles[switch_pos] != _X || hole_masked) ? 1 : 0;
    SBPuzzle<H, W>::move_tile(switch_pos);
    return cost;
}

template <int H, int W>
int SBPuzzle<H, W>::apply_action(ExpandedAction ma) {
    if(ma.new_tile_pos != hole_pos) // moving through multiple don't care tiles
        tiles[hole_pos] = _X;
    tiles[ma.new_tile_pos] = tiles[ma.new_hole_pos];
    tiles[ma.new_hole_pos] = HOLE;
    hole_pos = ma.new_hole_pos;
    return 1;
}

template <int H, int W>
bool SBPuzzle<H, W>::operator==(const SBPuzzle<H, W> &other) const {
    // a little hack here, if the hole is not in the group, all its adjacent
    // positions are equivalent, so we need to propagate the hole before
    // checking equality and then undo it
    bool equal = true;
    propagate_hole();
    other.propagate_hole();
    for(int i = 0; i < SIZE; i++) {
        if(tiles[i] != other.tiles[i]) {
            equal = false;
            break;
        }
    }
    unpropagate_hole();
    other.unpropagate_hole();
    return equal;
}

template <int H, int W>
std::ostream &operator<<(std::ostream &s, typename SBPuzzle<H, W>::Direction dir) {
    using Dir = typename SBPuzzle<H, W>::Direction;
    switch(dir) {
        case Dir::UP:       s << "U"; break;
        case Dir::RIGHT:    s << "R"; break;
        case Dir::LEFT:     s << "L"; break;
        case Dir::DOWN:     s << "D"; break;
        default:            s << "X"; break;
    }
    return s;
}

template <typename RandomAccessIterator>
static int count_inversions(RandomAccessIterator begin, RandomAccessIterator end) {
    // count inversions with O(n^2) instead of mergesort style,
    // since the vectors are very small
    int inversions = 0;
    int size = end - begin;
    int hole = size - 1;
    for(RandomAccessIterator i = begin; i != end; i++) {
        if(*i != hole) {
            for(RandomAccessIterator j = i+1; j != end; j++) {
                if(*i > *j) {
                    inversions++;
                }
            }
        }
    }
    return inversions;
}

template <int H, int W>
bool SBPuzzle<H, W>::is_solvable() const {
    // see https://www.cs.bham.ac.uk/~mdr/teaching/modules04/java2/TilesSolvability.html
    int inversions = count_inversions(tiles, tiles + SIZE);
    int hole_row_from_bottom = H - hole_pos % W;
    if(W % 2 == 1) // odd width
        return inversions % 2 == 0; // must have even inversions
    if(hole_row_from_bottom % 2 == 0) // even width
        return inversions % 2 == 1; // must have odd inversions
    // otherwise even
    return inversions % 2 == 1;
}

template <int H, int W>
template <typename Iterator>
void SBPuzzle<H, W>::apply_moves(Iterator begin, Iterator end) {
    for(Iterator itr = begin; itr != end; itr++)
        apply_action(*itr);
}

template <int H, int W>
int SBPuzzle<H, W>::find_hole() const {
    for(int i = 0; i < SIZE; i++)
        if(tiles[i] == HOLE)
            return i;
    throw std::invalid_argument("Puzzle does not contain a hole");
    return 0;
}

// add hashability
namespace std {
    template <int H, int W>
    struct hash<SBPuzzle<H, W>> {
        size_t operator()(SBPuzzle<H, W> const &p) const noexcept;
    };
}

template <int H, int W>
size_t std::hash<SBPuzzle<H, W>>::operator()(SBPuzzle<H, W> const &p) const noexcept {
    return p.hash();
}

template <int H, int W>
size_t SBPuzzle<H, W>::hash() const {
    // a little hack here, just like equality check 
    propagate_hole();
    // copied hash combination from the  boost implementation
    // first, perform a loop using 8 bytes every time
    size_t seed = 0;
    constexpr std::hash<uint64_t> hasher64;
    constexpr int view_size = SIZE >> 3;
    const uint64_t *tiles_view = reinterpret_cast<const uint64_t *>(tiles);
    for(int i = 0; i < view_size; i++)
        seed ^= hasher64(tiles_view[i]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    // and then do the rest 1 byte at a time
    constexpr std::hash<uint8_t> hasher8;
    constexpr int rem_start = view_size << 3;
    constexpr int rem_end = rem_start + SIZE - (view_size << 3);
    for(int i = rem_start; i < rem_end; i++)
        seed ^= hasher8(tiles[i]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    // of course, unmask before returning if necessary
    unpropagate_hole();
    return seed;
}

template <int H, int W>
int SBPuzzle<H, W>::manhattan_distance_to_solution() const {
    int dist = 0;
    for(int i = 0; i < SIZE; i++) {
        if(tiles[i] != _X && tiles[i] != HOLE) {
            int row = i / W, col = i % W;
            int actual_row = tiles[i] / W, actual_col = tiles[i] % W;
            dist += abs(row - actual_row) + abs(col - actual_col);
        }
    }
    return dist;
}

template <int H, int W>
void SBPuzzle<H, W>::propagate_hole() const {
    SBPuzzle<H, W> &mutable_this = const_cast<SBPuzzle &>(*this);
    mutable_this.propagate_hole();
}

template <int H, int W>
void SBPuzzle<H, W>::unpropagate_hole() const {
    SBPuzzle<H, W> &mutable_this = const_cast<SBPuzzle &>(*this);
    mutable_this.unpropagate_hole();
}

#endif
