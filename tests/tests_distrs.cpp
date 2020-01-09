
#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <functional>

#include "../sbpuzzle.hpp"
#include "../search.hpp"

static inline size_t num_digits(size_t x) {
    // another crude but simple approach, wouldn't work well for largest 64b ints and slow
    return x ? static_cast<size_t>(log10(static_cast<double>(x))) + 1 : 1;
}

template <typename ForwardItr>
void print_crude_int_hist(ForwardItr begin, ForwardItr end, size_t damp) {
    // build the histogram
    ForwardItr min, max;
    std::tie(min, max) = std::minmax_element(begin, end);
    int64_t nbins = *max - *min + 1;
    std::vector<int64_t> hist(nbins, 0);
    for(auto itr = begin; itr != end; ++itr)
        ++hist[*itr - *min];

    // print the columns
    size_t width = num_digits(*max);
    std::string fill_str(width, '#');
    std::string blank_str(width, ' ');
    int64_t hist_max = *std::max_element(hist.begin(), hist.end());
    size_t hwidth = num_digits(hist_max);
    for(int64_t lim = hist_max; lim > 0; lim -= damp) {
        bool prev_blank = false;
        std::cout << std::setw(hwidth) << std::setfill(' ') << lim << '|';
        for(int64_t i = 0; i < nbins; ++i) {
            if(hist[i] >= lim) {
                if(prev_blank) std::cout << '|';
                std::cout << fill_str << '|';
            } else {
                if(prev_blank) std::cout << ' ';
                std::cout << blank_str;
            }
            prev_blank = !(hist[i] >= lim);
        }
        std::cout << std::endl;
    }

    // print the base
    std::cout << std::string((width + 1) * (nbins + 1), '-') << std::endl;
    std::cout << std::string(hwidth, ' ');
    for(int64_t i = 0; i < nbins; ++i)
        std::cout << '|' << std::setw(width) << std::setfill(' ') << (*min + i);
    std::cout << '|' << std::endl;
}

static constexpr size_t N = 200000;
static constexpr int64_t sstart = 0, send = 30;

namespace search = denizmsayin::sblock::search;
using namespace denizmsayin::sblock::sbpuzzle;
using namespace generation;
using namespace heuristics;

std::string PDB_PATH = "/home/deniz//sblock/datafiles/pdbs/dp3x3_full.db";

int main() {
    constexpr uint8_t H = 3, W = 3;
    auto rng = std::default_random_engine(783);
    std::vector<Basic<H, W>> puzzles(N, Basic<H, W>::uninitialized());
    using URNG = decltype(rng);
    using OI = decltype(puzzles)::iterator;

    auto hval = heuristic_factory<H, W>(HeuristicType::CPDB, PDB_PATH);
    Heuristic<H, W> &heuristic = *hval;

    auto solver = search::search_factory<Basic<H, W>, TileSwapAction, Heuristic<H, W> &, pcost_t, true, true>(search::SearchType::ID_ASTAR);

    for(const std::string &stype_str : RANDOM_GENERATOR_STRINGS) {
        std::cout << stype_str << " scrambling " << sstart << "-" 
                  << send << ":" << std::endl;
        
        auto stype = str2randomgeneratortype(stype_str);
        auto gen = random_sbpuzzle_generator_factory<H, W, URNG, OI>(stype, sstart, send);
        gen(N, rng, puzzles.begin());
        
        std::vector<int> costs;
        for(size_t i = 0; i < N; ++i) {
            auto r = solver(puzzles[i], 0, 0, heuristic);
            costs.emplace_back(r.cost);
            std::cout << '\r' << "Solved: " << (i+1) << '/' << N << std::flush;
        }
        std::cout << std::endl;

        puzzles.clear();
        print_crude_int_hist(costs.begin(), costs.end(), N / 200);
    }

    std::cout << "Please check the crude histograms visually to confirm the intent of each distribution" << std::endl;

    return 0;
}
