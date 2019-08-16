#ifndef __SBPUZZLE_H__
#define __SBPUZZLE_H__

#include <iostream>
#include <vector>


class SBPuzzle {
public:
    // an enumeration for specifying actions on the puzzle
    enum class Direction {
        UP,
        RIGHT,
        DOWN,
        LEFT
    };

    // Initializes an hxw sliding block puzzle with a solved configuration 
    SBPuzzle(int h, int w);
    // Initializes an hxw sliding block puzzle with the provided configuration
    SBPuzzle(const std::vector<int> &tiles, int h, int w);

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
    void apply_moves(const std::vector<Direction> moves); 

    

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

    // return the position of the hole
    int find_hole() const;
};

#endif
