#ifndef __SBLOCK_UTILS_HPP__
#define __SBLOCK_UTILS_HPP__

#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <cstdint>

// first, non-templated functions 
void write_byte_array(const uint8_t *byte_array, size_t size, const char *filename);
void read_byte_array(uint8_t *byte_array, size_t size, const char *filename);
size_t combination(size_t x, size_t n);
size_t factorial(size_t n);
std::ostream& stream_tiles(std::ostream &s, const uint8_t *tiles, size_t h, size_t w, uint8_t x);

template <typename Iterator>
size_t calculate_combindex(Iterator begin, Iterator end, int x, int n);

template <typename RandomAccessIterator>
size_t calculate_lexindex(RandomAccessIterator begin, RandomAccessIterator end);

// templated class declarations
template <typename Arithmetic>
class SeriesTracker {
public:
    struct Options {
        std::ostream *stream;
        double alpha;
        Arithmetic print_every;
        bool show_speed;

        Options(int pe=1000, double a=0.0, bool ss=true, std::ostream *s=&std::cout) 
            : stream(s), alpha(a), print_every(pe), show_speed(ss) {}
    };

    SeriesTracker(const Arithmetic *to_track);
    SeriesTracker(const Arithmetic *to_track, const Options &opts);

    void track();


private:
    const Arithmetic *tracked;
    Arithmetic rec_value;
    std::chrono::time_point<std::chrono::high_resolution_clock> rec_time;
    Options options;
    double running_avg;
    static constexpr size_t STRING_BUF_SIZE = 100;

    void record();
};

// thin wrapper around integral types to allow for different printing 
template <typename A, bool = std::is_integral<A>::value>
struct NumWrapper {
    A x;

    NumWrapper(A y) : x(y) {}

};

template <typename A>
std::ostream& operator<<(std::ostream &os, const NumWrapper<A, true> &w);

template <typename A>
std::ostream& operator<<(std::ostream &os, const NumWrapper<A, false> &w);

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
size_t calculate_combindex(Iterator begin, Iterator end, int x, int n) {
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


// aims to add a single comma between each 3 digit:
// e.g. x = 1842532 -> 1,842,532
template <typename A>
std::ostream& operator<<(
        std::ostream &os, 
        const NumWrapper<A, true> &w)
{
    uint64_t y = w.x;
    if(w.x < 0) {
        os << "-";
        y = -y; // hope x is not int64_t min!
    }
    uint64_t mult = 1000000000000000000ULL; // largest mult for uint64_t
    bool reached = false;
    bool first_digit = true;
    while(mult > 0) {
        uint64_t div = y / mult;
        if(div > 0) // don't want to print anything until the first digit is reached
            reached = true;
        if(reached) {
            if(!first_digit) { // for the first set of digits, no leading zeros
                if(div < 100)
                    os << '0';
                if(div < 10)
                    os << '0';
                if(div < 1)
                    os << '0';
            }
            first_digit = false;
            if(div > 0)
                os << div;
            y -= mult * div;
            if(mult > 1) // no comma after the final value
                os << ",";
        }
        mult /= 1000;
    }
    return os;
}

template <typename A>
std::ostream& operator<<(
        std::ostream &os, 
        const NumWrapper<A, false> &w)
{
    os << w.x;
    return os;
}

template <typename A>
SeriesTracker<A>::SeriesTracker(const A *to_track) : tracked(to_track),
    rec_value(), rec_time(), options()
{
    record();
}

template <typename A>
SeriesTracker<A>::SeriesTracker(const A *to_track, const Options &opts) : 
    tracked(to_track), rec_value(), rec_time(), options(opts)
{
    record();
}

template <typename A>
void SeriesTracker<A>::record() {
    rec_value = *tracked;
    rec_time = std::chrono::high_resolution_clock::now();
}

template <typename A>
void SeriesTracker<A>::track() {
    A diff = *tracked - rec_value;
    if(diff >= options.print_every) {
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms = now - rec_time;
        double inst_speed = 1000 * diff / ms.count(); // per second
        running_avg = options.alpha * running_avg + (1 - options.alpha) * inst_speed;
        NumWrapper<A> print_value(*tracked);
        NumWrapper<double> print_avg(running_avg);
        if(options.show_speed)
            (*options.stream) << "Value: " << print_value << ", cps: " << print_avg << std::endl;
        else
            (*options.stream) << "Value: " << print_value << std::endl;
        record();
    }
}

#endif
