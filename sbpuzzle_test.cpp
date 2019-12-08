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

using namespace sbpuzzle;

#ifdef W_TORCH
#include "dlmodel.hpp"
const std::string torchfile = "/home/deniz/HDD/Documents/CENG783/Project/supervised/small_l2.pt";
auto device = torch::kCUDA;
std::unique_ptr<DLModel<H, W>> DLMODEL = DLModel<H, W>::from_file(torchfile, device);
#endif

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

/*
template <int H, int W, class URNG>
SBPuzzle<H, W> create_solvable_masked_puzzle(URNG &&rng) {
    SBPuzzle<H, W> puzzle;
    bool mask[H*W];
    for(int i = 0, size = H*W; i < size; ++i)
        mask[i] = true;
    mask[H*W-1] = false;
    do {
        puzzle.shuffle(rng);
    } while(!puzzle.is_solvable());
    return SBPuzzle<H, W>(puzzle, mask);
}
*/

class ZeroHeuristic {
public:
    constexpr uint64_t operator()(const Puzzle &p) {
        return 0;
    }
};

/*
template <int H, int W>
class MisplacedTilesHeuristic {
public:
    int operator()(const SBPuzzle<H, W> &p) {
        return p.num_misplaced_tiles();
    }
};
*/
/*
class ManhattanHeuristic {
public:
    uint64_t operator()(const Puzzle &p) {
        return p.manhattan_distance_to_solution();
    }
};

class DPDBHeuristic {
public:
    int operator()(const Puzzle &p) {
        return static_cast<const PDB<H, W> *>(&CDB)->lookup(p);
    }
};
*/

size_t gequal = 0;
std::vector<int64_t> goffs;

#ifdef W_TORCH
class TorchHeuristic {
public:
    uint64_t operator()(const Puzzle &p) {
        uint64_t r = DLMODEL->forward(p);
        /* auto ar = search2::a_star_search<Puzzle, TSA, DPDBHeuristic>(p);
        int64_t actual = ar.cost;
        goffs.emplace_back(actual - r);
        if(r == actual)
            ++gequal;
        */
        return r;
    }
};

class BTorchHeuristic {
public:
    template <typename OutItr>
    void operator()(const std::vector<Puzzle> &p, OutItr i) {
        DLMODEL->forward(p, i);
        /* auto ar = search2::a_star_search<Puzzle, TSA, DPDBHeuristic>(p);
        int64_t actual = ar.cost;
        goffs.emplace_back(actual - r);
        if(r == actual)
            ++gequal;
        */
    }
};
#endif

/*
bool check_equality(const vector<Dir> &m1, const vector<Dir> &m2) {
    bool eq = false;
    if(m1.size() == m2.size()) {
        int size = m1.size();
        eq = true;
        for(int i = 0; i < size; i++)
            if(m1[i] != m2[i]) {
                eq = false;
                break;
            }
    }
    return eq;
}
*/

void show_usage() {
    std::cout 
        << "Usage: ./exe_file [-h] [-r seed] [-n num-puzzles] [-s search-type] [-h heuristic]\n"
        << "                  [-f file] [-w weight] [-b batch_size] [-g]\n"
        << "       -?             show this help message\n"
        << "       -r SEED        seed value used to generate puzzles, default is 42\n"
        << "       -n NUM_PUZZLES the number of puzzles to randomly generate, default is 1\n"
        << "       -s SEARCH_TYPE the type of search to use: {bfs, iddfs, a*, ida*, wa*, bwa*}\n"
        << "                      default is bfs\n"
        << "       -h HEURISTIC   heuristic to use: {mispl, manhattan, pdb, rdb, cdb, dlmodel}\n"
        << "                      mispl: number of misplaced tiles in the puzzle\n"
        << "                      manhattan: manhattan distance to the solved state\n"
        << "                      pdb: disjoint pattern database (from file)\n"
        << "                      rdb: disjoint pattern database, reflected\n"
        << "                      cdb: disjoint pattern database, combined with reflection\n"
        << "                      dlmodel: TorchScript deep learning model\n"
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

enum class SearchType {
    BFS,
    IDDFS,
    ASTAR,
    ID_ASTAR,
    WEIGHTED_ASTAR,
    BATCH_WEIGHTED_ASTAR
};

SearchType str2searchtype(const std::string &s) {
    typedef SearchType T;
    static std::unordered_map<std::string, SearchType> map {
        {"bfs", T::BFS},
        {"iddfs", T::IDDFS},
        {"a*", T::ASTAR},
        {"ida*", T::ID_ASTAR},
        {"wa*", T::WEIGHTED_ASTAR},
        {"bwa*", T::BATCH_WEIGHTED_ASTAR}
    };
    return map[s];
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

    int ch;
    while((ch = getopt(argc, argv, "?r:n:s:h:f:w:b:g")) != -1) {
        switch(ch) {
            case '?':   show_usage(); return 0;
            case 'r':   seed = std::stoul(optarg); break;
            case 'n':   num_puzzles = std::stoull(optarg); break;
            case 's':   search_type_str = optarg; break;
            case 'h':   heuristic_str = optarg; break;
            case 'f':   file_path = optarg; break;
            case 'w':   weight = std::stod(optarg); break;
            case 'b':   batch_size = std::stoull(optarg); break;
            case 'g':   use_gpu = true;
            default:    std::cerr << "Unexpected argument. Try ./exe_file -?\n"; return -1;
        }
    }

    // create n random puzzles
    auto rng = std::default_random_engine(seed);
    std::vector<Puzzle> puzzles;
    for(size_t i = 0; i < num_puzzles; i++) 
        puzzles.push_back(create_solvable_puzzle<H, W>(rng));

    using namespace sbpuzzle;
    SearchType search_type = str2searchtype(search_type_str);
    HeuristicType heuristic = str2heuristictype(heuristic_str);

    // the search functions require virtual dispatch of the heuristic function,
    // however since the templates are written with value (and not pointer) types
    // in mind, it is necessary to use a reference to Heuristic rather than a pointer
    std::unique_ptr<Heuristic<H, W>> hp = heuristic_factory<H, W>(heuristic);
    Heuristic<H, W> &hf = *hp;

    auto t1 = std::chrono::high_resolution_clock::now();
    int num_moves = 0;
    size_t num_nodes = 0;
    for(size_t i = 0; i < num_puzzles; i++) {
        using namespace search2;
        auto r = a_star_search<Puzzle, TSA>(puzzles[i], hf);
        // auto r = batch_weighted_a_star_search<Puzzle, TSA>(puzzles[i], 1.0, 1, hf);
        // auto r = search2::batch_weighted_a_star_search<Puzzle, TSA, decltype(bhf)>(puzzles[i], 1.0, 16, bhf);

        // auto r = search2::weighted_a_star_search<Puzzle, TSA, DPDBHeuristic>(puzzles[i], 0.7);
        // auto r = search2::batch_weighted_a_star_search<Puzzle, TSA, BTorchHeuristic, true>(puzzles[i], 1.0, 8192);
        // auto r = search2::weighted_a_star_search<Puzzle, TSA, TorchHeuristic, true>(puzzles[i], 0.1);
        // std::cout << "Solved in " << m << " moves." << std::endl;
        num_moves += r.cost;
        // num_moves += search2::iterative_deepening_a_star<Puzzle, TSA, DPDBHeuristic>(puzzles[i]);
        // num_moves += search2::a_star_search<Puzzle, TSA, ManhattanHeuristic>(puzzles[i]);
        // num_moves += moves.size();
        num_nodes += r.nodes_expanded; // search2::get_node_counter<Puzzle>();
        // search2::reset_node_counter<SBPuzzle<H, W>>();
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

//    cout << "Equals: " << gequal << '/' << goffs.size() << "\n";

    return 0;
}
