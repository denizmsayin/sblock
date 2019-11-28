#ifndef __SBLOCK_UTILS_HPP__
#define __SBLOCK_UTILS_HPP__

#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <cstdint>

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

public:
};

// thin wrapper around integral types to allow for different printing 
template <typename A, bool = std::is_integral<A>::value>
struct NumWrapper {
    A x;

    NumWrapper(A y) : x(y) {}

};

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
