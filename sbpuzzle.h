#ifndef __SBPUZZLE_H__
#define __SBPUZZLE_H__

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>

class SBPuzzle {
public:
    // an enumeration for specifying actions on the puzzle
    enum class Direction {
        UP,
        RIGHT,
        DOWN,
        LEFT,
        INVALID
    };

    // Initializes an hxw sliding block puzzle with a solved configuration 
    explicit SBPuzzle(int h, int w);
    // Initializes an hxw sliding block puzzle with the provided configuration
    explicit SBPuzzle(const std::vector<int> &tiles, int h, int w);

    SBPuzzle(const SBPuzzle &other) = default;
    SBPuzzle(SBPuzzle &&other) = default;

    // returns True if the puzzle is in a solved state
    bool is_solved() const;

    // return the position of the hole
    int find_hole() const;

    // apply the move in the given direction using the cached hole position
    // and return the new hole position
    int apply_move(Direction move, int hole_pos);

    // return true if the puzzle is part of the solvable permutations
    bool is_solvable() const;

    // shuffle the puzzle randomly
    template <class URNG>
    void shuffle(URNG &&urng) { std::shuffle(tiles.begin(), tiles.end(), urng); }

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
    void apply_moves(const std::vector<Direction> &moves); 

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
    friend std::ostream &operator<<(std::ostream &s, const SBPuzzle &p);

private:
    std::vector<int> tiles;
    int h;
    int w;

    // encode the puzzle into a string for hashing, not as good as making your own hash,
    // but a shortcut. Will have to try the alternative...
    std::string encode() const;
};

std::ostream &operator<<(std::ostream &s, SBPuzzle::Direction dir);

#endif
