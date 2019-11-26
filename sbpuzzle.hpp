#ifndef __SBPUZZLE_H__
#define __SBPUZZLE_H__

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <cstdint>

// an enumeration for specifying actions on the puzzle
enum class Direction : uint8_t {
    UP,
    RIGHT,
    DOWN,
    LEFT,
    INVALID
};

template <int H, int W>
class SBPuzzle {
public:

    // Initializes an hxw sliding block puzzle with a solved configuration
    explicit SBPuzzle();
    // Initializes an hxw sliding block puzzle with the provided configuration
    explicit SBPuzzle(const int tiles[]);
    template <class Iterator>
    explicit SBPuzzle(Iterator begin, Iterator end);

    SBPuzzle(const SBPuzzle &other) = default;
    SBPuzzle(SBPuzzle &&other) = default;

    // return the goal state for this puzzle
    SBPuzzle goal_state() const;

    // returns True if the puzzle is in a solved state
    bool is_solved() const;

    // returns the possible actions on the puzzle
    const std::vector<Direction> &possible_actions() const;

    // apply the move in the given direction using the cached hole position
    // return the path cost, which is 1 for our case for any choice
    int apply_action(Direction move);

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

    // Optimally solves the puzzle using breadth first search. Returns the ordered list
    // of moves to be done to reach the solution, without modifying the original
    std::vector<Direction> solution_bfs() const;

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

    // encode the puzzle into a string for hashing, not as good as making your own hash,
    // but a shortcut. Will have to try the alternative...
    std::string encode() const;

    size_t hash() const;

    // heuristic for misplaced tiles
    int num_misplaced_tiles() const;

    // heuristic for manhattan distance
    int manhattan_distance_to_solution() const;



private:
    uint8_t tiles[H*W];
    uint8_t hole_pos; // cached

    int find_hole() const;

    constexpr static int SIZE = H * W;
    constexpr static int HOLE = H * W - 1;
};

// add hashability
namespace std {
    template <int H, int W>
    struct hash<SBPuzzle<H, W>> {
        size_t operator()(SBPuzzle<H, W> const &p) const noexcept;
    };
}

Direction inverse(Direction d) {
    using Dir = Direction;
    switch(d) {
        case Dir::UP:       return Dir::DOWN;
        case Dir::RIGHT:    return Dir::LEFT;
        case Dir::DOWN:     return Dir::UP;
        case Dir::LEFT:     return Dir::RIGHT;
        default:            return Dir::INVALID;
    }
}

template <int H, int W>
SBPuzzle<H, W>::SBPuzzle() : hole_pos(HOLE)
{
    for(int i = 0; i < SIZE; i++)
        tiles[i] = i;
}

template <int H, int W>
SBPuzzle<H, W>::SBPuzzle(const int o_tiles[])
{
    for(int i = 0; i < SIZE; i++) {
        if(o_tiles[i] == HOLE)
            hole_pos = i;
        tiles[i] = o_tiles[i];
    }
}

template <int H, int W>
template <typename Iterator>
SBPuzzle<H, W>::SBPuzzle(Iterator begin, Iterator end)
{
    int *itr = tiles;
    while(begin != end)
        *itr++ = *begin++;
}

template <int H, int W>
SBPuzzle<H, W> SBPuzzle<H, W>::goal_state() const {
    return SBPuzzle<H, W>();
}

template <int H, int W>
bool SBPuzzle<H, W>::is_solved() const {
    for(int i = 0; i < SIZE; i++)
        if(tiles[i] != i)
            return false;
    return true;
}

template <int H, int W>
const std::vector<Direction> &SBPuzzle<H, W>::possible_actions() const {
    static std::vector<Direction> actions;
    actions.resize(4);
    int i = 0;
    if(hole_pos >= W) // not upper edge, can move UP
        actions[i++] = Direction::UP;
    if(hole_pos % W < W - 1) // not right edge, can move RIGHT
        actions[i++] = Direction::RIGHT;
    if(hole_pos < SIZE - W) // not lower edge, can move DOWN
        actions[i++] = Direction::DOWN;
    if(hole_pos % W > 0) // not left edge, can move LEFT
        actions[i++] = Direction::LEFT;
    actions.resize(i);
    return actions;
}

template <int H, int W>
int SBPuzzle<H, W>::apply_action(Direction move) {
    int switch_pos;
    switch(move) {
        case Direction::UP:    switch_pos = hole_pos - W; break;
        case Direction::RIGHT: switch_pos = hole_pos + 1; break;
        case Direction::DOWN:  switch_pos = hole_pos + W; break;
        case Direction::LEFT:  switch_pos = hole_pos - 1; break;
        default:               throw std::invalid_argument("Invalid move in move vector");
    }
    tiles[hole_pos] = tiles[switch_pos];
    tiles[switch_pos] = HOLE;
    hole_pos = switch_pos; // the new hole position
    return 1; // the path cost
}

template <int H, int W>
bool SBPuzzle<H, W>::operator==(const SBPuzzle<H, W> &other) const {
    if(hole_pos == other.hole_pos) {
        for(int i = 0; i < SIZE; i++) {
            if(tiles[i] != other.tiles[i])
                return false;
        }
        return true;
    }
    return false;
}

template <int H, int W>
std::ostream &operator<<(std::ostream &s, Direction dir) {
    using Dir = Direction;
    switch(dir) {
        case Dir::UP:       s << "U"; break;
        case Dir::RIGHT:    s << "R"; break;
        case Dir::LEFT:     s << "L"; break;
        case Dir::DOWN:     s << "D"; break;
        default:            s << "X"; break;
    }
    return s;
}

template <int H, int W>
std::ostream &operator<<(std::ostream &s, const SBPuzzle<H, W> &p) {
    int num_dashes = W * 4 + 1;
    std::string dash_str(num_dashes, '-');
    for(int i = 0, k = 0; i < H; i++) {
        s << dash_str << std::endl;
        for(int j = 0; j < W; j++)
            s << "| " << p.tiles[k++] << " ";
        s << "|" << std::endl;
    }
    s << dash_str;
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

// Calculates the lexicographical index of a permutation
// e.g. reference 0 1 2 3
//      0 1 2 3 -> 0
//      0 1 3 2 -> 1
//      1 0 2 3 -> 6 etc.
//      3 2 1 0 -> 23
template <typename RandomAccessIterator>
static size_t calculate_lexindex(RandomAccessIterator begin, RandomAccessIterator end) {
    // Once again, we use the O(n^2) approach since it is faster for the small arrays
    // that we intend to deal with
    if(end - begin <= 1LL) // arrays with 0 or 1 size
        return 0;
    size_t counter = 0;
    size_t mult = 1;
    for(RandomAccessIterator i = end - 2; i >= begin; i--) {
        for(RandomAccessIterator j = i + 1; j != end; j++)
            if(*i > *j)
                counter += mult;
        mult *= (end - i); // mult starts from 1!, becomes 2!, 3!, 4! etc.
    }
    return counter;
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

// TODO: figure out a nice index calculation
template <int H, int W>
size_t std::hash<SBPuzzle<H, W>>::operator()(SBPuzzle<H, W> const &p) const noexcept {
    return p.hash();
}

template <int H, int W>
size_t SBPuzzle<H, W>::hash() const {
    // return calculate_lexindex(tiles, tiles + SIZE);
    // copied from the  boost implementation
    size_t seed = 0;
    std::hash<uint8_t> hasher;
    for(int i = 0; i < SIZE; i++)
        seed ^= hasher(tiles[i]) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    return seed;
}

template <int H, int W>
std::string SBPuzzle<H, W>::encode() const {
    // an important assumption here is that the puzzle has less than 256 tiles, which is a given
    // since we cannot solve puzzles larger than 5x5
    std::string encoded;
    encoded.reserve(SIZE);
    for(int i = 0; i < SIZE; i++)
        encoded.push_back(tiles[i]);
    return encoded;
}

template <int H, int W>
int SBPuzzle<H, W>::manhattan_distance_to_solution() const {
    int dist = 0;
    for(int i = 0; i < SIZE; i++) {
        if(tiles[i] != HOLE) {
            int row = i / W, col = i % W;
            int actual_row = tiles[i] / W, actual_col = tiles[i] % W;
            dist += abs(row - actual_row) + abs(col - actual_col);
        }
    }
    return dist;
}

template <int H, int W>
int SBPuzzle<H, W>::num_misplaced_tiles() const {
    int n = 0;
    for(int i = 0; i < SIZE; i++)
        if(tiles[i] != HOLE && tiles[i] != i)
            n++;
    return n;
}

#endif
