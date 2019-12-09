#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <chrono>
#include <string>
#include <numeric>
#include <memory>
#include <unordered_map>
#include <unistd.h>

#include "search2.hpp"
#include "sbpuzzle.hpp"
#include "pdb.hpp"
#include "dpdb.hpp"
#include "reflectdpdb.hpp"
#include "combineddb.hpp"
#include "heuristics.hpp"

#ifndef __H
#define __H 3
#endif
#ifndef __W
#define __W 3
#endif

// program constants
constexpr uint8_t H = __H, W = __W;

//-------------------------------------------------------------------------------

typedef sbpuzzle::TileSwapAction TSA;
typedef sbpuzzle::SBPuzzle<H, W> Puzzle;

template <int H, int W, class URNG>
Puzzle create_solvable_puzzle(URNG &&rng) {
    std::array<uint8_t, H*W> tiles;
    std::iota(tiles.begin(), tiles.end(), 0);
    do {
        std::shuffle(tiles.begin(), tiles.end(), rng); 
    } while(!sbpuzzle::tiles_solvable<H, W>(tiles));
    return Puzzle(tiles);
}

// takes a vector {"1", "2", "3"} and returns a string representation "{1, 2, 3}"
std::string make_string_list(const std::vector<std::string> &strings) {
    if(strings.empty()) return "{}";
    std::string acc = std::accumulate(strings.begin()+1, strings.end(), 
                                      std::string("{") + strings[0],
                                      [](const auto &s1, const auto &s2) { 
                                          return s1 + ", " + s2;
                                      });
    return acc + "}";
}

void show_usage() {
    std::string sstring = make_string_list(search2::SEARCH_STRINGS);
    std::string hstring = make_string_list(sbpuzzle::HEURISTIC_STRINGS);
    std::cout 
        << "Usage: ./exe_file [-h] [-r seed] [-n num-puzzles] [-s search-type] [-h heuristic]\n"
        << "                  [-f file] [-w weight] [-b batch_size] [-g]\n"
        << "       -?             show this help message\n"
        << "       -r SEED        seed value used to generate puzzles, default is 42\n"
        << "       -n NUM_PUZZLES the number of puzzles to randomly generate, default is 1\n"
        << "       -s SEARCH_TYPE the type of search to use: " << sstring << "\n"
        << "                      default is bfs\n"

    // auto-print heuristic strings from those registered in heuristics.hpp
        << "       -h HEURISTIC   heuristic to use: " << hstring << "\n";
    for(const auto &k : sbpuzzle::HEURISTIC_STRINGS) {
        auto itr = sbpuzzle::HEURISTIC_DESCR_MAP.find(k);
        std::cout 
        << "                      " << itr->first << ": " << itr->second << "\n";
    }

    std::cout 
        << "                      only relevant if the search type is one of the A* types\n"
        << "                      default is manhattan\n"
        << "       -f FILE        file to load data from, only necessary for the pdb/rdb/cdb\n"
        << "                      and dlmodel heuristics; must be provided\n"
        << "       -w WEIGHT      a value k in the range [0, 1], used for weighting A* search\n"
        << "                      in the form f = k*g + f (lowering relevance of path cost)\n"
        << "                      only relevant for wa* and bwa*, default value is 1.0, which\n"
        << "                      is actually the same as standard A* search.\n"
        << "       -b BATCH_SIZE  batch size for bwa*, important when using a dlmodel on gpu\n"
        << "                      default is 1\n"
        << "       -g             stands for GPU: if set, CUDA is enabled for dlmodels\n"
        ;
}


int main(int argc, char *argv[]) {
    unsigned seed = 42;
    size_t num_puzzles = 1;
    std::string search_type_str = "bfs";
    std::string heuristic_str = "manhattan";
    std::string file_path;
    double weight = 1.0;
    size_t batch_size = 1;
    bool use_gpu = false;

    if(argc == 1)
        std::cout << "For argument help: ./exe_file -?\n";

    int ch;
    while((ch = getopt(argc, argv, "?gr:n:s:h:f:w:b:")) != -1) {
        switch(ch) {
            case '?':   show_usage(); return 0;
            case 'r':   seed = std::stoul(optarg); break;
            case 'n':   num_puzzles = std::stoull(optarg); break;
            case 's':   search_type_str = optarg; break;
            case 'h':   heuristic_str = optarg; break;
            case 'f':   file_path = optarg; break;
            case 'w':   weight = std::stod(optarg); break;
            case 'b':   batch_size = std::stoull(optarg); break;
            case 'g':   use_gpu = true; break;
            default:    std::cerr << "Unexpected argument (" << static_cast<char>(ch) << "). " 
                                  << "Try ./exe_file -?\n"; return -1;
        }
    }

    // create n random puzzles
    auto rng = std::default_random_engine(seed);
    std::vector<Puzzle> puzzles;
    for(size_t i = 0; i < num_puzzles; i++) 
        puzzles.push_back(create_solvable_puzzle<H, W>(rng));

    using namespace sbpuzzle;
    using namespace search2;

    HeuristicType heuristic;
    try {
        heuristic = str2heuristictype(heuristic_str);
    } catch(const std::out_of_range &ex) {
        std::cerr << "Error: invalid heuristic type string.\n";
        return -1;
    }

    // the search functions require virtual dispatch of the heuristic function,
    // however since the templates are written with value (and not pointer) types
    // in mind, it is necessary to use a reference to Heuristic rather than a pointer
    std::unique_ptr<Heuristic<H, W>> hp = heuristic_factory<H, W>(heuristic, file_path, use_gpu);
    Heuristic<H, W> &hf = *hp;
    
    SearchType search_type;
    try {
        search_type = str2searchtype(search_type_str);
    } catch(const std::out_of_range &ex) {
        std::cerr << "Error: invalid search type string.\n";
        return -1;
    }

    auto search_function = search_factory<Puzzle, TSA, Heuristic<H, W> &, true>(search_type);

    auto t1 = std::chrono::high_resolution_clock::now();
    int num_moves = 0;
    size_t num_nodes = 0;
    for(size_t i = 0; i < num_puzzles; i++) {
        using namespace search2;
        auto r = search_function(puzzles[i], weight, batch_size, std::ref(hf));
        num_moves += r.cost;
        num_nodes += r.nodes_expanded;
        std::cout << '\r' << (i+1) << "/" << num_puzzles << std::flush;
    }
    
    std::cout << '\r';
    auto t2 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

    double avg_moves = static_cast<double>(num_moves) / num_puzzles;
    double avg_nodes = static_cast<double>(num_nodes) / num_puzzles;
    std::cout << "Solved " << num_puzzles << " puzzles in " << fp_ms.count() / num_puzzles 
              << " milliseconds on average with " << avg_moves << " moves and " << avg_nodes
              << " nodes expanded on average." << std::endl;

    return 0;
}
