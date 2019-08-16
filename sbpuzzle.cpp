#include "sbpuzzle.h"

#include <iostream>
#include <string>
#include <stdexcept>
#include <vector>
#include <list>
#include <queue>
#include <unordered_set>

SBPuzzle::SBPuzzle(int h, int w) : tiles(h*w), h(h), w(w)
{
    int size = h * w;
    for(int i = 0; i < size; i++)
        tiles[i] = i;
}

SBPuzzle::SBPuzzle(const std::vector<int> &tiles, int h, int w) : tiles(tiles), h(h), w(w) {}

bool SBPuzzle::is_solved() const {
    int size = h * w;
    for(int i = 0; i < size; i++)
        if(tiles[i] != i)
            return false;
    return true;
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

int SBPuzzle::apply_move(Direction move, int hole_pos) {
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
    return switch_pos; // the new hole position
}

void SBPuzzle::apply_moves(const std::vector<Direction> &moves) {
    int hole_pos = find_hole();
    for(Direction d : moves) 
        hole_pos = apply_move(d, hole_pos);
}


// Record structure stored in the queue for breadth first search
struct BfsRec {
    SBPuzzle puzzle;
    const BfsRec *prev;
    SBPuzzle::Direction last_move;
    int hole_pos;

    // Initialization for the first state
    BfsRec(const SBPuzzle &p) : puzzle(p), prev(nullptr), 
        last_move(SBPuzzle::Direction::INVALID), hole_pos(p.find_hole()) {}
    // Initialization for following states
    BfsRec(const SBPuzzle &p, const BfsRec *rec, SBPuzzle::Direction move, int hole_pos) : 
        puzzle(p), prev(rec), last_move(move), hole_pos(hole_pos) {}
};

// create a new record by moving from 'prev_rec' to direction 'dir', and add it to 'records'
// also enqueue the new record for processing
static void chain_new_record(std::queue<BfsRec *> &q,
                             std::list<BfsRec> &records,
                             const BfsRec *prev_rec,
                             SBPuzzle::Direction dir)
{
    SBPuzzle new_puzzle(prev_rec->puzzle);
    int new_hole_pos = new_puzzle.apply_move(dir, prev_rec->hole_pos);
    records.emplace_back(
        new_puzzle,
        prev_rec,
        dir,
        new_hole_pos
    );
    q.push(&records.back());
}

static std::vector<SBPuzzle::Direction> reconstruct_moves(const BfsRec *final_record) {
    std::vector<SBPuzzle::Direction> moves;
    for(const BfsRec *rec = final_record; rec->prev != nullptr; rec = rec->prev)
        moves.push_back(rec->last_move);
    std::reverse(moves.begin(), moves.end());
    return moves;
}

std::vector<SBPuzzle::Direction> SBPuzzle::solution_bfs() const {
    int size = h * w; // size of the puzzle

    // Instead of having only a queue, we have both a list and a queue. The vector
    // acts as a global list for the records. We need this because we do not want records
    // to be erased when they are dequeued, since each record points to the previous
    // record so that we can reconstruct the set of moves that leads to the solution.
    // The queue stores pointers to use less space, and records are a list instead of 
    // vector so as not to invalidate pointers on reallocation.
    std::list<BfsRec> records;
    std::queue<BfsRec *> q;

    std::unordered_set<std::string> visited; // set to remember visited states
    
    records.emplace_back(*this);
    q.push(&records.back());
    while(!q.empty()) {
        BfsRec *rec = q.front(); 
        q.pop();

        // encode the puzzle to hash it and check if it was already visited
        std::string encoded = rec->puzzle.encode();
        if(visited.find(encoded) == visited.end()) { 

            // reconstruct and return the path if the puzzle is solved
            if(rec->puzzle.is_solved())
                return reconstruct_moves(rec);

            // add the state to visited states
            visited.insert(encoded);

            // generate new states depending on the possibilities
            int hole_pos = rec->hole_pos;
            if(hole_pos >= w) // not upper edge, can move UP
                chain_new_record(q, records, rec, Direction::UP);
            if(hole_pos % w < w - 1) // not right edge, can move RIGHT
                chain_new_record(q, records, rec, Direction::RIGHT);
            if(hole_pos < size - w) // not lower edge, can move DOWN
                chain_new_record(q, records, rec, Direction::DOWN);
            if(hole_pos % w > 0) // not left edge, can move LEFT
                chain_new_record(q, records, rec, Direction::LEFT);

        }
    }

    return std::vector<Direction>(); // empty vector, no solution
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
