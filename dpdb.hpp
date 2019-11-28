#ifndef __DPDB_HPP__
#define __DPDB_HPP__

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>

#include "masked_sbpuzzle.hpp"
#include "search2.hpp"

// An implementation of disjoint pattern databases for the sliding block puzzle

// a quick function to determine combination values by table lookup
constexpr size_t MAX_COMB = 50;
static size_t combination(size_t x, size_t n) {
    static size_t table[MAX_COMB][MAX_COMB+1] = {0}; // remember for dynamic programming
    if(x == n || n == 0)
        return 1;
    else if(table[x][n] == 0)
        table[x][n] = combination(x-1, n-1) + combination(x-1, n);
    return table[x][n];
}

// another similar function for factorial values
constexpr size_t MAX_FACTORIAL = 50;
static size_t factorial(size_t n) {
    static size_t table[MAX_FACTORIAL] = {0};
    if(n == 0)
        return 1;
    else if(table[n] == 0)
        table[n] = n * factorial(n-1);
    return table[n];
}

// Now, we need a function to calculate the index of combination. Examples:
// 0: XXXOOO XXXOO XXOOO XXXXO XOOOO XXXXX OOOOO
// 1: XXOXOO XXOXO XOXOO XXXOX OXOOO
// 2: XXOOXO XXOOX XOOXO XXOXX OOXOO
// 3: XXOOOX XOXXO XOOOX XOXXX OOOXO
// 4: XOXXOO XOXOX OXXOO OXXXX OOOOX
// 5: XOXOXO XOOXX OXOXO
// 6: XOXOOX OXXXO OXOOX
// 7: XOOXXO OXXOX OOXXO
// 8: XOOXOX OXOXX OOXOX
// 9: XOOOXX OOXXX OOOXX
//10: OXXXOO
//11: OXXOXO
//12: OXXOOX
//13: OXOXXO
//14: OXOXOX
//15: OXOOXX
//16: OOXXXO
//17: OOXXOX
//18: OOXOXX
//19: OOOXXX
template <typename Iterator>
static size_t calculate_combindex(Iterator begin, Iterator end, int x, int n) {
    // x and n are required to know which combinations to use beforehand
    // the iterator should be for booleans, true means full (X), false means empty (Y)
    size_t counter = 0;
    while(begin != end) {
        if(*begin) // X
            --n;
        else if(n > 0) // O, and some X yet remain 
            counter += combination(x-1, n-1);
        ++begin;
        --x;
    }
    return counter;
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
    for(RandomAccessIterator i = end - 2; i >= begin; --i) {
        for(RandomAccessIterator j = i + 1; j != end; ++j)
            if(*i > *j)
                counter += mult;
        mult *= (end - i); // mult starts from 1!, becomes 2!, 3!, 4! etc.
    }
    return counter;
}


// Now, since our function that determines a combination index does so
// 'generically' over an iterator that returns booleans, we need to
// create a custom iterator class constructed over a set of tiles
// that presents a 'boolean' view of them
template <int H, int W>
class TileCombIterator {
public:
    // IMPORTANT: the iterator does not perform a copy of the input
    // tiles and simply uses the raw interpreted pointer. This is
    // why the tiles shouldn't change during the lifetime of a single
    // TileCombIterator
    // inp_tiles is simply the tile configuration as per sbpuzzle.hpp
    // in_group is an array, which shows true if a given tile is in the group
    // e.g. if we have a 3x3 puzzle and out group is {0 2 3 5}
    // in_group = [t f t t f t f f f]
    // Also, equality checks only compare the index, so it's up to the
    // user to ensure that tiles are the same
    TileCombIterator(const bool *inp_in_group, const uint8_t *inp_tiles, size_t index) : 
        in_group(inp_in_group), tiles(inp_tiles), i(index) {}

    TileCombIterator(const TileCombIterator &other) = default;
    TileCombIterator(TileCombIterator &&other) = default;
    TileCombIterator& operator=(const TileCombIterator &other) = default;
    TileCombIterator& operator=(TileCombIterator &&other) = default;

    const TileCombIterator &operator++() { // prefix ++
        i++;
        return *this;
    }

    bool operator*() const {
        return in_group[tiles[i]];
    }

    bool operator==(const TileCombIterator &other) const {
        return i == other.i;
    }

    bool operator!=(const TileCombIterator &other) const {
        return i != other.i;
    }

private:
    const uint8_t *tiles;
    const bool *in_group;
    size_t i;
};

template <int H, int W>
class DPDB {
public:
    DPDB() = default;

    // groups is supposed to contain a group for each tile
    // e.g. for 3x3 and 2 groups : {0, 0, 0, 1, 1, 1, 2, 2, 0}
    // Note that the group of the last element (hole) is ignored,
    // and that the groups MUST be enumerated from 0 to N-1
    template <typename GIterator>
    DPDB(GIterator groups_begin, GIterator groups_end);
    template <typename GIterator, typename FIterator>
    DPDB(GIterator groups_begin, GIterator groups_end, FIterator filenames_begin, FIterator filenames_end);
    ~DPDB();

    // function for a set of groups
    template <typename GIterator, typename FIterator>
    static void generate_and_save(GIterator groups_begin, GIterator groups_end, filenames_begin, Iterator filenames_end);
    
    // function for a single group given by a mask
    template <typename GIterator>
    static void _generate_and_save(GIterator mask_begin, GIterator mask_end, const char *filename);

    int lookup(const uint8_t *tiles) const;
//private:
    uint8_t **tables;
    uint8_t groups[H*W];
    uint8_t num_groups;
    uint8_t *group_counts;
    bool **in_groups;

    constexpr static int SIZE = H * W;
    constexpr static int HOLE = H * W - 1;

    TileCombIterator<H, W> boolview_begin(int group_num, const uint8_t *tiles) const {
        return TileCombIterator<H, W>(in_groups[group_num], tiles, 0);
    }

    TileCombIterator<H, W> boolview_end(int group_num, const uint8_t *tiles) const {
        return TileCombIterator<H, W>(in_groups[group_num], tiles, SIZE);
    }

    template <class OutputIterator>
    void fill_group(int group_num, const uint8_t *tiles, OutputIterator itr) const;
    
    template <typename GIterator>
    void init(GIterator groups_begin, GIterator groups_end);
};

// How are the tables laid out? 
// -> For each group, there is a table with C(Ntiles, Ngroup) * Ngroup! entries
// e.g. for a group with 7 elements in the 15 puzzle C(16, 7) * 7! = 57,657,600
// -> To index each table, we first have to determine where the blocks belonging to
// the group are, as the first part of the index (the C(Ntiles, Ngroup) part). 
// -> For the second index we simply calculate the lexicographical index of the ordering
// of the members of the group (the Ngroup! part)

template <int H, int W>
template <typename GIterator>
void DPDB<H, W>::init(GIterator groups_begin, GIterator groups_end) {
// copy the group number of each tile
    std::copy(groups_begin, groups_end, groups);
    // find the number of groups, assuming the largest element is it
    num_groups = *std::max_element(groups, groups + SIZE - 1) + 1;
    // count each group's elements and mark them
    // also allocate empty tables
    group_counts = new uint8_t[num_groups]();
    in_groups = new bool *[num_groups];
    for(uint8_t i = 0; i < num_groups; ++i)
        in_groups[i] = new bool [SIZE]();
    for(int i = 0; i < HOLE; ++i) {
        group_counts[groups[i]]++;
        in_groups[groups[i]][i] = true;
    } // NOTE: the hole is not in any group
    // initialize empty tables
    tables = new uint8_t *[num_groups];
    for(uint8_t i = 0; i < num_groups; ++i) {
        size_t db_size = combination(SIZE, group_counts[i]) * factorial(group_counts[i]);
        tables[i] = new uint8_t[db_size];
    }
}

template <int H, int W>
template <typename GIterator>
DPDB<H, W>::DPDB(GIterator groups_begin, GIterator groups_end) {
    init(groups_begin, groups_end);
}

template <int H, int W>
template <typename GIterator, typename FIterator>
DPDB<H, W>::DPDB(
        GIterator groups_begin, 
        GIterator groups_end, 
        FIterator filenames_begin, 
        FIterator filenames_end) 
{
    init(groups_begin, groups_end);
    // TODO: tables have to be read from each file
}

template <int H, int W>
DPDB<H, W>::~DPDB() {
    delete[] group_counts;
    for(uint8_t i = 0; i < num_groups; ++i) {
        delete[] tables[i];
        delete[] in_groups[i];
    }
    delete[] tables;
    delete[] in_groups;
}

// This function is necessary for easy use of the calculate_lexindex
// function. As all tiles in a group have to be in a contiguous array
// for their index to be calculated, this function simply fills
// the output iterator with consecutive tiles from the same group
template <int H, int W>
template <class OutputIterator>
void DPDB<H, W>::fill_group(int group_num, const uint8_t *tiles, OutputIterator itr) const {
    for(int i = 0; i < SIZE; ++i)
        if(in_groups[group_num][tiles[i]])
            *itr++ = tiles[i];
}

template <int H, int W>
template <typename GIterator, typename FIterator>
void DPDB<H, W>::_generate_and_save(
        GIterator mask_begin, 
        GIterator mask_end, 
        const char *filename) 
{
    using Dir = Direction;
    // we simply need to apply bfs, but only add a cost when a tile
    // from the group is moved, otherwise the moves do not cost anything
    bool mask[H*W];
    std::copy(mask_begin, mask_end, mask);
    MaskedSBPuzzle<H, W> p(SBPuzzle<H, W>::goal_state(), mask);
    // perform breadth first search
    search2::BreadthFirstIterator<MaskedSBPuzzle<H, W>, Dir> itr(p);
    while(!itr.done()) {
        ;
    }
}

#include <iostream>
template <int H, int W>
int DPDB<H, W>::lookup(const uint8_t *tiles) const {
    int total = 0;
    // for each group
    for(uint8_t i = 0; i < num_groups; ++i) {
        // first, find the combination index
        auto s = boolview_begin(i, tiles);
        auto e = boolview_end(i, tiles);
        size_t comb_i = calculate_combindex(s, e, SIZE, group_counts[i]);
        // then the lexicographical index of the group elements
        uint8_t group_size = group_counts[i];
        uint8_t group_tiles[group_size];
        fill_group(i, tiles, group_tiles);
        size_t lex_i = calculate_lexindex(group_tiles, group_tiles + group_size);
        // calculate the total index and lookup the table
        size_t index = comb_i * factorial(group_size) + lex_i;
        total += tables[i][index];
    }
    return total;
}


int main() {
    uint8_t groups[] = {0, 0, 1, /**/ 0, 1, 1, /**/ 0, 1, 0};
    // uint8_t tiles[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t tiles[] = {1, 8, 2, 3, 6, 5, 4, 7, 0};
    DPDB<3, 3> db(groups, groups + 9);
    db.lookup(tiles);
    return 0;
}

#endif
