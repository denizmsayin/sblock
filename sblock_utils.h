#ifndef __SBLOCK_UTILS_H__
#define __SBLOCK_UTILS_H__

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

#endif
