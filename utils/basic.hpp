#ifndef DENIZMSAYIN_SBLOCK_UTILS_BASIC_HPP
#define DENIZMSAYIN_SBLOCK_UTILS_BASIC_HPP

#include <iostream>
#include <iomanip>
#include <array>
#include <vector>
#include <chrono>
#include <string>
#include <cstdint>
#include <cctype>

// first, non-templated functions 
// implementations for these are contained in .cpp

// calculate combinations & factorial
unsigned long combination(unsigned long x, unsigned long n);
unsigned long factorial(unsigned long n);

// hash byte arrays by 8 bytes at each step, and combine with boost's
// implementation of hash combination
size_t hash_byte_array(const uint8_t *a, size_t s);

// generate a number of bit strings that are different, useful for zobrist
// hashing. returns a vector because it is used for static initialization.
std::vector<size_t> generate_different_bitstrings(size_t num_bit_strings, unsigned seed);

// calculate combination & lexical indices over iterators
template <typename Iterator>
size_t calculate_combindex(Iterator begin, Iterator end, unsigned long x, unsigned long n);

template <typename RandomAccessIterator>
size_t calculate_lexindex(RandomAccessIterator begin, RandomAccessIterator end);

// one hot encoding, for dlmodels
template <typename T1, typename T2>
void one_hot_encode(const T1 *inp, size_t s, T2 *out);

template <typename T1, typename T2>
void one_hot_encode(const T1 *inp, size_t s, T2 *out) {
    std::fill(out, out + s*s, static_cast<T2>(0));
    for(size_t i = 0; i < s; ++i)
        out[i * s + static_cast<size_t>(inp[i])] = static_cast<T2>(1);
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
size_t calculate_combindex(Iterator begin, Iterator end, unsigned long x, unsigned long n) {
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
size_t calculate_lexindex(RandomAccessIterator begin, RandomAccessIterator end) {
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

#endif
