#ifndef __DPDB_HPP__
#define __DPDB_HPP__

#include <algorithm>
#include <cstdint>
#include <fstream>

// An implementation of disjoint pattern databases for the sliding block puzzle

template <int H, int W>
class DPDB {
public:
    template <typename Iterator, typename... Args>
    DPDB(Iterator groups_begin, Iterator groups_end, Args&&... filenames);

    int lookup(const uint8_t *tiles) const;
private:
    uint8_t **tables;
    uint8_t groups[H*W];
    uint8_t num_groups;

    constexpr static int SIZE = H * W;
    constexpr static int HOLE = H * W - 1;
};

// How are the tables laid out? 
// -> For each group, there is a table with C(Ntiles, Ngroup) * Ngroup! entries
// e.g. for a group with 7 elements in the 15 puzzle C(16, 7) * 7! = 57,657,600
// -> To index each table, we first have to determine where the blocks belonging to
// the group are, as the first part of the index (the C(Ntiles, Ngroup) part). 
// -> For the second index we simply calculate the lexicographical index of the ordering
// of the members of the group (the Ngroup! part)

// a quick function to determine combination values by table lookup
constexpr size_t MAX_COMB = 50;
static int combination(size_t x, size_t n) {
    static int table[MAX_COMB][MAX_COMB+1] = {0}; // remember for dynamic programming
    if(x == n || n == 0)
        return 1;
    else if(table[x][n] == 0)
        table[x][n] = combination(x-1, n-1) + combination(x-1, n);
    return table[x][n];
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
            n--;
        else if(n > 0) // O, and some X yet remain 
            counter += combination(x-1, n-1);
        begin++;
        x--;
    }
    return counter;
}

#include <iostream>
int main() {
    using namespace std;
    bool t = true, f = false;
    bool arr[] = {f, f, t, t, f};
    cout << calculate_combindex(arr, arr+5, 5, 2) << endl;
    return 0;
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
int DPDB<H, W>::lookup(const uint8_t *tiles) const {
    // first, we have to generate 
}


#endif
