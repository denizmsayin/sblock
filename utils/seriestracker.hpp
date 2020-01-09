#ifndef DENIZMSAYIN_SBLOCK_UTILS_SERIESTRACKER_HPP
#define DENIZMSAYIN_SBLOCK_UTILS_SERIESTRACKER_HPP

#include <iostream>

namespace denizmsayin::sblock::utils {

    // templated class declarations
    template <typename Arithmetic>
    class SeriesTracker {
    public:
        class Options {
        public:
            Options()
                : _do_track{true}, _stream{std::cout}, _alpha{0.0}, _print_every{1}, 
                  _show_speed{true}, _name_str{"Value"}, _no_newline{true}, _target_value{0},
                  _clear_on_destruction{false} {}

            // builder-like pattern for large objects
            // only 'like', because properties can be changed
            // after construction as well
            Options &do_track(bool b) { _do_track = b; return *this; }
            Options &stream(std::ostream &s) { _stream = s; return *this; }
            Options &alpha(double a) { _alpha = a; return *this; }
            Options &print_every(Arithmetic p) { _print_every = p; return *this; }
            Options &show_speed(bool ss) { _show_speed = ss; return *this; }
            Options &name_str(const std::string &ns) { _name_str = ns; return *this; }
            Options &no_newline(bool nn) { _no_newline = nn; return *this; }
            Options &target_value(Arithmetic tv) { _target_value = tv; return *this; }
            Options &clear_on_destruction(bool c) { _clear_on_destruction = c; return *this; }

            Options(const Options &o) = default;
            Options(Options &&o) = default;

        private:
            bool _do_track;
            std::ostream &_stream;
            double _alpha;
            Arithmetic _print_every;
            bool _show_speed;
            std::string _name_str;
            bool _no_newline;
            Arithmetic _target_value;
            bool _clear_on_destruction;

            friend class SeriesTracker;
        };

        SeriesTracker() = default;
        SeriesTracker(const Arithmetic *to_track);
        SeriesTracker(const Arithmetic *to_track, const Options &opts);
        ~SeriesTracker();

        void track();


    private:
        const Arithmetic *tracked;
        Arithmetic rec_value;
        std::chrono::time_point<std::chrono::high_resolution_clock> rec_time;
        Options options;
        double running_avg;
        bool did_print;
        static constexpr size_t STRING_BUF_SIZE = 100;

        void record();
    };

    template <typename Arithmetic>
    class SeriesTrackedValue {
    public:
        using Options = typename SeriesTracker<Arithmetic>::Options;

        SeriesTrackedValue(Arithmetic initial_value) : value(initial_value), tracker(&value) {}
        SeriesTrackedValue(Arithmetic initial_value, 
                           const Options &opts)
            : value(initial_value), tracker(&value, opts) {}

       
        Arithmetic get_value() const { return value; }

        // TODO: add many more overloads to make this like a transparent numeric type

        // do not support chained assignment
        void operator+=(Arithmetic added) {
            value += added;
            tracker.track();
        }

        void operator-=(Arithmetic subbed) {
            value -= subbed;
            tracker.track();
        }

        void operator*=(Arithmetic mul) {
            value *= mul;
            tracker.track();
        }

        void operator/=(Arithmetic div) {
            value /= div;
            tracker.track();
        }

        void operator++()       { *this += 1; }
        void operator++(int)    { *this += 1; }
        void operator--()       { *this -= 1; }
        void operator--(int)    { *this -= 1; }

    private:
        Arithmetic value;
        SeriesTracker<Arithmetic> tracker;
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
    SeriesTracker<A>::SeriesTracker(const A *to_track, const Options &opts) : 
        tracked(to_track), rec_value(), rec_time(), options(opts), running_avg(), did_print(false)
    {
        record();
    }

    template <typename A>
    SeriesTracker<A>::SeriesTracker(const A *to_track) : SeriesTracker(to_track, Options()) {}

    template <typename A>
    SeriesTracker<A>::~SeriesTracker() {
        // TODO: count the number of chars pushed by the last print for clearing
        const static std::string clear_str = std::string(70, ' ');
        // clear the stream when going out of scope
        if(did_print && options._no_newline) {
            if(options._clear_on_destruction) 
                options._stream << clear_str << '\r';
            else
                options._stream << std::endl;
        }
    }

    template <typename A>
    void SeriesTracker<A>::record() {
        rec_value = *tracked;
        rec_time = std::chrono::high_resolution_clock::now();
    }

    template <typename A>
    void SeriesTracker<A>::track() {
        if(options._do_track && options._print_every != static_cast<A>(0)) {
            A diff = *tracked - rec_value;
            if(diff >= options._print_every) {
                did_print = true; // mark for clearing
                auto now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> ms = now - rec_time;
                double inst_speed = 1000 * diff / ms.count(); // per second
                running_avg = options._alpha * running_avg + (1 - options._alpha) * inst_speed;
                NumWrapper<A> print_value(*tracked);
                NumWrapper<double> print_avg(running_avg);
                NumWrapper<A> target_value(options._target_value);
                char fc = tolower(options._name_str[0]);
                if(options._no_newline) 
                    options._stream << '\r';
                options._stream << options._name_str << ": " << print_value;
                if(options._target_value != 0)
                    options._stream << '/' << target_value;
                if(options._show_speed)
                    options._stream << ", " << fc << "ps: " << print_avg;
                if(options._no_newline)
                    options._stream << std::flush;
                else
                    options._stream << std::endl;
                record();
            }
        }
    }

}

#endif
