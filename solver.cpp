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
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <unistd.h>

#include "search2.hpp"
#include "sbpuzzle.hpp"
#include "sbpuzzle_generation.hpp"
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

constexpr size_t WRITER_THREAD_PRINT_EVERY = 1000000;

//-------------------------------------------------------------------------------

// typedef sbpuzzle::TileSwapAction Action;
typedef sbpuzzle::DirectionAction Action;
typedef sbpuzzle::SBPuzzle<H, W> Puzzle;
typedef uint8_t cost_t;

// compact struct to store puzzle solutions
struct Solution {
    Puzzle puzzle;
    cost_t cost;

    Solution(const Puzzle &p, cost_t c) : puzzle(p), cost(c) {}
};

// global variables & syncs
std::queue<Puzzle> gpuzzles;
std::mutex m_gpuzzles;

std::queue<Solution> gsolutions;
std::mutex m_gsolutions;
std::condition_variable cv_enough_solutions;
std::atomic<size_t> gpuzzles_to_solve;

std::atomic<size_t> gnum_moves;
std::atomic<size_t> gnum_nodes;

template <typename Solver, typename HeuristicEvaluator>
void solver_thread_routine(Solver solver, double w, size_t b, HeuristicEvaluator heuristic) {
    while(true) {
        // dequeue a puzzle from the generated ones if possible
        Puzzle p;
        {
            std::scoped_lock lock(m_gpuzzles);
            // TODO: try to make this work with an initiall empty queue
            // without making it too complicated
            if(gpuzzles.empty()) break; // no more puzzles to consume, quit the loop
            p = gpuzzles.front(); // dequeue a puzzle
            gpuzzles.pop();
        }
        // solve the puzzle using A* search
        search2::SearchResult r = solver(p, w, b, heuristic);
        // enqueue the solution and wake up the writer if necessary
        {
            std::scoped_lock lock(m_gsolutions);
            gsolutions.emplace(p, r.cost);
        }
        gnum_moves += r.cost;
        gnum_nodes += r.nodes_expanded;
        --gpuzzles_to_solve; // decrement puzzle count
        // wake the writer since a solution has been found
        cv_enough_solutions.notify_one();
    }
}

void writer_thread_routine(std::fstream &stream, size_t print_every) {
    // initialize a tracker for printing progress
    size_t written_sols = 0;
    SeriesTracker<size_t>::Options opts;
    opts.print_every = print_every;
    opts.name_str = "puzzles solved";
    SeriesTracker<size_t> sol_tracker(&written_sols, opts);
    
    // actual variables & logic
    std::queue<Solution> sols; // local queue for copying
    bool done = false;
    while(!done) {
        {
            // wait on wake 
            std::unique_lock lock(m_gsolutions);
            cv_enough_solutions.wait(lock, []{ return gpuzzles_to_solve == 0 
                    || !gsolutions.empty(); });

            done = (gpuzzles_to_solve == 0);
            // TODO: is there a better way to unlock early?
            // empty into the local queue
            while(!gsolutions.empty()) {
                sols.emplace(gsolutions.front());
                gsolutions.pop();
            }
            // can now unlock gsolutions
        }

        // the lock on solutions is unlocked, can write safely
        while(!sols.empty()) {
            const Solution &s = sols.front();
            s.puzzle.to_binary_stream(stream) << s.cost;
            stream.flush();
            sols.pop();
            // track the number of solutions written
            ++written_sols;
            sol_tracker.track();
        }
    }
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
    std::string pstring = make_string_list(sbpuzzle::RANDOM_GENERATOR_STRINGS);

    auto print_descrs = [](const std::vector<std::string> &strings,
                           const std::unordered_map<std::string, std::string> &descrs)
    {
        for(const auto &k : strings)
            std::cout 
            << "                      " << k << ": " 
                                        << descrs.at(k) << "\n";
    };

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

        << "       -h HEURISTIC   heuristic to use: " << hstring << "\n";
        print_descrs(sbpuzzle::HEURISTIC_STRINGS, sbpuzzle::HEURISTIC_DESCR_MAP);
    std::cout 
        << "                      only relevant if the search type is one of the A* types\n"
        << "                      default is manhattan\n";

    std::cout
        << "       -p PUZZLEGEN   puzzle generation method to use: " << pstring << "\n";
        print_descrs(sbpuzzle::RANDOM_GENERATOR_STRINGS, sbpuzzle::RANDOM_GENERATOR_DESCR_MAP);
    std::cout
        << "                      default is shuffle.\n"
        << "       -l LOWER       lower limit on the number of moves to apply during puzzle\n"
        << "                      generation if using a scrambling generator. Defaults to 1.\n"
        << "       -u UPPER       lower limit on the number of moves to apply during puzzle\n"
        << "                      generation if using a scrambling generator. Defaults to 50.\n";

    std::cout
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
        << "       -j NJOBS       number of worker threads to use to solve puzzles, with an \n"
        << "                      extra thread for writing the solutions to a file if necessary.\n"
        << "                      Defaults to 0 which is the single threaded implementation.\n";
        ;
}


int main(int argc, char *argv[]) {
    unsigned seed = 42;
    size_t num_puzzles = 1;
    std::string search_type_str = "bfs";
    std::string heuristic_str = "manhattan";
    std::string gen_str = "shuffle";
    std::string file_path;
    std::string in_path;
    std::string out_path;
    double weight = 1.0;
    size_t batch_size = 1;
    bool use_gpu = false;
    int64_t lower = 1;
    int64_t upper = 50;
    size_t num_threads = 0;

    search2::TRACKER_OPTS.do_track = true;

    if(argc == 1)
        std::cout << "For argument help: ./exe_file -?\n";

    int ch;
    while((ch = getopt(argc, argv, "?gr:n:s:h:f:w:b:i:o:p:l:u:j:")) != -1) {
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
            case 'p':   gen_str = optarg; break;
            case 'l':   lower = std::stoll(optarg); break;
            case 'u':   upper = std::stoll(optarg); break;      
            case 'j':   num_threads = std::stoull(optarg); break;
            default:    std::cerr << "Unexpected argument (" << static_cast<char>(ch) << "). " 
                                  << "Try ./exe_file -?\n"; return -1;
        }
    }


    using namespace sbpuzzle;
    // create n random puzzles
    std::vector<Puzzle> puzzles;

    if(in_path == "") { // generate puzzles
        RandomGeneratorType generator_type;
        try {
            generator_type = str2randomgeneratortype(gen_str);
        } catch(const std::out_of_range &ex) {
            std::cerr << "Error: invalid generator type string.\n";
            return -1;
        }

        auto rng = std::default_random_engine(seed);
        using RNG = decltype(rng);
        using OutItr = decltype(std::back_inserter(puzzles));
        auto puzzle_generator = random_sbpuzzle_generator_factory<H, W, RNG, OutItr>(
            generator_type, lower, upper);
        puzzle_generator(num_puzzles, rng, std::back_inserter(puzzles));
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

    // open the output file and check how many bytes it has
    bool outfile_exists = (out_path != "");
    std::fstream outfile; 
    if(outfile_exists) {

        outfile.open(out_path, std::fstream::in | std::fstream::out 
                               | std::fstream::app | std::ofstream::binary);
        if(!outfile.is_open()) {
            std::cerr << "Error: could not open output file.\n";
            return -1;
        }

        outfile.ignore(std::numeric_limits<std::streamsize>::max()); // read until EOF
        size_t file_size = outfile.gcount();
        outfile.clear();

        // if the file is not new, must be continuing from a previous run
        // fill an already_solved set from the file
        if(file_size > 0) {
            // check how many instances the output file contains
            size_t storage_size = Puzzle::tile_size_in_bytes() + sizeof(Solution::cost);
            size_t file_num_puzzles = file_size / storage_size;
            size_t rem = file_size % storage_size;
            if(rem != 0) {
                std::cout << "Error: output file size is not a multiple of puzzle size" << std::endl;
                return -1;
            }
            std::cout << "Outfile is an existing file containing " << file_num_puzzles
                      << " solved instances." << std::endl;

            // quit if it was already filled
            if(file_num_puzzles >= num_puzzles) {
                std::cout << "The outfile already contains at least as many puzzles as requested."
                          << " Quitting." << std::endl;
                return 0;
            }
            
            // fill the already_solved set, counting how many of each puzzle exist
            std::unordered_map<Puzzle, int> already_solved;
            outfile.seekg(std::fstream::beg);
            for(size_t i = 0; i < num_puzzles; ++i) { // remove already solved puzzles
                std::array<uint8_t, H*W> tiles; // TODO: puzzle::underlying_type ?
                outfile.read(reinterpret_cast<char *>(&tiles[0]), tiles.size());
                outfile.ignore(1); // discard 1 byte (the solution)
                Puzzle tile_puzzle(tiles);
                auto lookup = already_solved.find(tile_puzzle);
                if(lookup == already_solved.end())
                    already_solved.emplace(tiles, 1);
                else
                    already_solved[tile_puzzle] += 1;
            }

            // next, we have to remove already solved puzzles from the generated ones
            // this is painful since they're in a vector; the approach is to mask those
            // that should be removed, and then move all unmasked ones to a new puzzle vector
            std::vector<int8_t> should_keep(num_puzzles, 1);
            for(size_t i = 0; i < num_puzzles; ++i) {
                auto lookup = already_solved.find(puzzles[i]);
                if(lookup != already_solved.end() && lookup->second > 0) {
                    --lookup->second;
                    should_keep[i] = 0;
                }
            }

            std::vector<Puzzle> new_puzzles;
            for(size_t i = 0; i < num_puzzles; ++i)
                if(should_keep[i])
                    new_puzzles.push_back(puzzles[i]);
            std::swap(puzzles, new_puzzles);
            num_puzzles = puzzles.size();

            std::cout << "Puzzles filtered by those already present in the output file." << std::endl;
            outfile.clear(); // reset the outfile for writing
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

        // set counters to zero
        gnum_moves = 0;
        gnum_nodes = 0;


        if(num_threads == 0) { // single threaded implementation
            for(size_t i = 0, size = puzzles.size(); i < size; ++i) {
                auto r = search_function(puzzles[i], weight, batch_size, std::ref(hf));
                gnum_moves += r.cost;
                gnum_nodes += r.nodes_expanded;
                std::cout << '\r' << (i+1) << "/" << num_puzzles << std::flush;
                writer(puzzles[i], r.cost);
            }
            std::cout << '\r';
        } else {
            // initialize variables
            gpuzzles_to_solve = num_puzzles;
    
            for(const auto &puzzle : puzzles)
                gpuzzles.push(puzzle);
            puzzles.clear();

            // spawn worker threads
            std::cout << "Spawning threads..." << std::endl;
            using Solver = decltype(search_function);
            std::vector<std::thread> worker_threads(num_threads);
            for(auto &thread : worker_threads)
                thread = std::thread(solver_thread_routine<Solver, Heuristic<H, W> &>, search_function, weight, batch_size, std::ref(hf));

            // set the printing amount

            // the main program will take care of writing
            if(outfile_exists) {
                std::cout << "Starting outfile writer..." << std::endl;
                outfile.clear();
                writer_thread_routine(std::ref(outfile), WRITER_THREAD_PRINT_EVERY);
            }

            // join worker threads
            std::cout << "Joining threads..." << std::endl;
            for(auto &thread : worker_threads)
                thread.join();


        }

        if(outfile_exists)    
            outfile.close();
        auto t2 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

        double avg_moves = static_cast<double>(gnum_moves) / num_puzzles;
        double avg_nodes = static_cast<double>(gnum_nodes) / num_puzzles;
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
