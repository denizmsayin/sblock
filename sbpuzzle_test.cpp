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

// TODO: merge this and generate_solutions.cpp into a single epic frontend

// program constants
constexpr uint8_t H = __H, W = __W;
const std::string EVAL_HEURISTIC_STR = "H";

//-------------------------------------------------------------------------------

// typedef sbpuzzle::TileSwapAction Action;
typedef sbpuzzle::DirectionAction Action;
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
    std::vector<std::string> ext(search2::SEARCH_STRINGS);
    ext.emplace_back(EVAL_HEURISTIC_STR); // empty string for no search
    std::string sstring = make_string_list(ext);
    std::string hstring = make_string_list(sbpuzzle::HEURISTIC_STRINGS);
    std::cout 
        << "Usage: ./exe_file [-?] [-r seed/source] [-n num-puzzles] [-s search-type]\n"
        << "                  [-h heuristic] [-f file] [-w weight] [-b batch_size] [-g]\n"
        << "                  [-i input_file] [-o output_file]\n"
        << "       -?             show this help message\n"
        << "       -r SEED        seed value used to generate puzzles, default is 42\n"
        << "       -n NUM_PUZZLES the number of puzzles to randomly generate, default is 1\n"
        << "       -s SEARCH_TYPE the type of search to use: " << sstring << "\n"
        << "                      default is bfs. " << EVAL_HEURISTIC_STR << " implies\n"
        << "                      generating heuristic values, rather than solving the puzzle.\n"

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
        << "       -i INPUT_FILE  interpreted as a path to an input binary file containing\n"
        << "                      puzzle-cost pairs (see generate_solutions.cpp). puzzles\n"
        << "                      will be taken from this file rather than being randomly\n"
        << "                      generated. Overrides -r and -n switches. \n"
        << "       -o OUTPUT_FILE output file to save results in binary format as puzzle-cost\n" 
        << "                      pairs (same format as source files). none by default.\n"
        ;
}


int main(int argc, char *argv[]) {
    unsigned seed = 42;
    size_t num_puzzles = 1;
    std::string search_type_str = "bfs";
    std::string heuristic_str = "manhattan";
    std::string file_path;
    std::string in_path;
    std::string out_path;
    double weight = 1.0;
    size_t batch_size = 1;
    bool use_gpu = false;

    search2::TRACKER_OPTS.do_track = true;

    if(argc == 1)
        std::cout << "For argument help: ./exe_file -?\n";

    int ch;
    while((ch = getopt(argc, argv, "?gr:n:s:h:f:w:b:i:o:")) != -1) {
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
            case 'i':   in_path = optarg; break;
            case 'o':   out_path = optarg; break;
            default:    std::cerr << "Unexpected argument (" << static_cast<char>(ch) << "). " 
                                  << "Try ./exe_file -?\n"; return -1;
        }
    }

    // create n random puzzles
    std::vector<Puzzle> puzzles;

    if(in_path == "") { // generate puzzles
        auto rng = std::default_random_engine(seed);
        for(size_t i = 0; i < num_puzzles; i++) 
            puzzles.emplace_back(create_solvable_puzzle<H, W>(rng));
    } else { // read puzzles from file
        // TODO: make this more elegant by using Puzzle::from_binary_stream
        // currently it is not very practical because it is not suited to failure,
        // i.e. reading the final puzzle (without using infile.peek())
        num_puzzles = 0;
        std::array<uint8_t, H*W> tiles;
        char *tiles_ptr = reinterpret_cast<char *>(tiles.data());
        size_t tiles_size = tiles.size() * sizeof(tiles[0]);
        std::ifstream infile(in_path, std::ifstream::in | std::ifstream::binary);
        if(!infile.is_open()) {
            std::cerr << "Error: could not open input file.\n";
            return -1;
        }
        while(infile.read(tiles_ptr, tiles_size)) { // while tiles can be read
            puzzles.emplace_back(tiles); // insert the puzzle 
            infile.ignore(1); // ignore the stored cost character
            ++num_puzzles;
        }
        infile.close();
        std::cout << "Read " << num_puzzles << " puzzles from file " << in_path << std::endl;
    }

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

    // create a function to write the output
    bool outfile_exists = (out_path != "");
    std::ofstream outfile; 
    if(outfile_exists) {
        outfile.open(out_path, std::ofstream::out | std::ofstream::binary);
        if(!outfile.is_open()) {
            std::cerr << "Error: could not open output file.\n";
            return -1;
        }
    }
    auto writer = [&](const Puzzle &p, uint8_t cost) {
        if(outfile_exists)
            p.to_binary_stream(outfile) << cost;
    };


    if(search_type_str != EVAL_HEURISTIC_STR) {
        SearchType search_type;
        try {
            search_type = str2searchtype(search_type_str);
        } catch(const std::out_of_range &ex) {
            std::cerr << "Error: invalid search type string.\n";
            return -1;
        }

        auto search_function = search_factory<Puzzle, Action, Heuristic<H, W> &>(search_type);

        auto t1 = std::chrono::high_resolution_clock::now();
        int num_moves = 0;
        size_t num_nodes = 0;
        for(size_t i = 0, size = puzzles.size(); i < size; ++i) {
            using namespace search2;
            auto r = search_function(puzzles[i], weight, batch_size, std::ref(hf));
            num_moves += r.cost;
            num_nodes += r.nodes_expanded;
            std::cout << '\r' << (i+1) << "/" << num_puzzles << std::flush;
            writer(puzzles[i], r.cost);
        }
 
        std::cout << '\r';
        auto t2 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

        double avg_moves = static_cast<double>(num_moves) / num_puzzles;
        double avg_nodes = static_cast<double>(num_nodes) / num_puzzles;
        std::cout << "Solved " << num_puzzles << " puzzles in " << fp_ms.count() / num_puzzles 
                  << " milliseconds on average with " << avg_moves << " moves and " << avg_nodes
                  << " nodes expanded on average." << std::endl;

    } else {
        // use the batch version of the heuristic
        std::vector<int> costs;
        hf(puzzles, costs);
        int total_cost = std::accumulate(costs.begin(), costs.end(), 0);
        if(outfile_exists) // small check to prevent wasting time
            for(size_t i = 0, size = puzzles.size(); i < size; ++i)
                writer(puzzles[i], costs[i]);
        std::cout << "Evaluated " << num_puzzles << " puzzles, resulting in an average of "
                  << static_cast<double>(total_cost) / num_puzzles << " heuristic cost."
                  << std::endl;
    }
   
    return 0;
}
