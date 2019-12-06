#include <iostream>
#include <array>
#include <vector>
#include <queue>
#include <random>
#include <numeric>
#include <unordered_set>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fstream>
#include <string>
#include <limits>

#include "sblock_utils.hpp"
#include "search2.hpp"
#include "sbpuzzle.hpp"
#include "pdb.hpp"
#include "crdpdb.hpp"

#ifndef __H
#define __H 3
#endif
#ifndef __W
#define __W 3
#endif

// program constants
constexpr uint8_t H = __H, W = __W;

constexpr unsigned SEED = 42;

constexpr size_t DEFAULT_WORKER_THREAD_COUNT = 0;
constexpr size_t DEFAULT_PRINT_EVERY_COUNT = 1000;

// typedefs for practicality & readability
typedef sbpuzzle::SBPuzzleWHole<H, W> Puzzle;
typedef sbpuzzle::TileSwapAction Action;
typedef sbpuzzle::PDB<H, W> PDB;
typedef sbpuzzle::CRDPDB<H, W> CRDPDB;

// compact struct to store puzzle solutions
struct Solution {
    Puzzle puzzle;
    uint8_t cost;

    Solution(const Puzzle &p, uint8_t c) : puzzle(p), cost(c) {}
};

// global variables & syncs
std::queue<Puzzle> gpuzzles;
std::mutex m_gpuzzles;

std::queue<Solution> gsolutions;
std::mutex m_gsolutions;
std::condition_variable cv_enough_solutions;
std::atomic<size_t> gpuzzles_to_solve;

// specialize std::hash

template <class URNG>
void generate_puzzles(size_t n, 
                      std::queue<Puzzle> &puzzles, 
                      const std::unordered_set<Puzzle> &already_solved,
                      URNG &&rng) {
    typedef std::array<uint8_t, H*W> Tiles;
    Tiles tiles;
    std::iota(tiles.begin(), tiles.end(), 0);
    std::unordered_set<Puzzle> generated;
    size_t i = 0;
    while(n--) {
        i++;
        bool success = false;
        while(!success) {
            // first shuffle the puzzle and see if it is solvable
            std::shuffle(tiles.begin(), tiles.end(), rng);
            if(!sbpuzzle::tiles_solvable<H, W>(tiles)) continue;
            // if it is, check if it has already been generated before
            // if not, we're done generating this puzzle 
            Puzzle p(tiles);
            if(generated.find(p) == generated.end()) { 
                success = true;
                generated.emplace(p);
                // only actually enqueue the generated puzzle if
                // it was not one that was previously solved
                // (only when resuming from a partially generated file)
                if(already_solved.find(p) == already_solved.end())
                    puzzles.emplace(p);
            }
        }
    }
}

void solver_thread_routine(const PDB *db) {
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
        search2::SearchResult r = 
            search2::a_star_search<Puzzle, Action>(p, [&](const auto &p) { 
                    return p.lookup_cost(db);
            });
        // enqueue the solution and wake up the writer if necessary
        {
            std::scoped_lock lock(m_gsolutions);
            gsolutions.emplace(p, r.cost);
        }
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
            s.puzzle.stream_binary(stream) << s.cost;
            stream.flush();
            sols.pop();
            // track the number of solutions written
            ++written_sols;
            sol_tracker.track();
        }
    }
}

int main(int argc, char *argv[]) {
    if(argc < 4 || argc > 6) {
        std::cout << "Usage: ./generate_solutions num_puzzles db_file out_file "
                  << "[worker_threads=" << DEFAULT_WORKER_THREAD_COUNT << "] " 
                  << "[print_every=" << DEFAULT_PRINT_EVERY_COUNT << "]" 
                  << std::endl;
        return 0;
    }

    size_t n = std::stoull(argv[1]);

    // open the output file and check how many bytes it has
    std::fstream stream(argv[3], std::fstream::in | std::fstream::out 
                                 | std::fstream::app | std::fstream::binary);
    stream.ignore(std::numeric_limits<std::streamsize>::max()); // read until EOF
    size_t file_size = stream.gcount();
    stream.clear();

    // if the file is not new, must be continuing from a previous run
    // fill an already_solved set from the file
    std::unordered_set<Puzzle> already_solved;
    if(file_size > 0) {
        // check how many instances the output file contains
        size_t storage_size = Puzzle::tile_size_in_bytes() + sizeof(Solution::cost);
        size_t num_puzzles = file_size / storage_size;
        size_t rem = file_size % storage_size;
        if(rem != 0) {
            std::cout << "Error: output file size is not a multiple of puzzle size" << std::endl;
            return 255;
        }
        std::cout << "Outfile is an existing file containing " << num_puzzles
                  << " solved instances." << std::endl;

        // quit if it was already filled
        if(num_puzzles >= n) {
            std::cout << "The outfile already contains at least as many puzzles as requested."
                      << " Quitting." << std::endl;
            return 0;
        }
        
        // fill the already_solved set
        stream.seekg(std::fstream::beg);
        for(size_t i = 0; i < num_puzzles; ++i) { // remove already solved puzzles
            std::array<uint8_t, H*W> tiles; // TODO: puzzle::underlying_type ?
            stream.read(reinterpret_cast<char *>(&tiles[0]), tiles.size());
            stream.ignore(1); // discard 1 byte (the solution)
            already_solved.emplace(tiles);
        }
    }

    // initialize the rng and generate puzzles
    auto rng = std::default_random_engine(SEED);
    generate_puzzles(n, gpuzzles, already_solved, rng);

    gpuzzles_to_solve = gpuzzles.size();

    // read the database from the provided file
    std::cout << "Reading database... ";
    CRDPDB db = CRDPDB::from_file(argv[2]);
    std::cout << "Done!" << std::endl;

    // set worker thread count
    size_t worker_thread_count = DEFAULT_WORKER_THREAD_COUNT;
    if(argc >= 5)
        worker_thread_count = std::stoull(argv[4]);

    // spawn worker threads
    std::vector<std::thread> worker_threads(worker_thread_count);
    for(auto &thread : worker_threads)
        thread = std::thread(solver_thread_routine, &db);

    // set the printing amount
    size_t print_every = DEFAULT_PRINT_EVERY_COUNT;
    if(argc >= 6) 
        print_every = std::stoull(argv[5]);
    if(worker_thread_count == 0) {
        std::cout << "W: print_every value ignored due to there being "
                  << "zero worker threads." << std::endl;
        print_every = 0;
    }

    if(worker_thread_count == 0) {
        // run in the main program
        std::cout << "W: 0 worker threads means that the output will only "
                  << "be saved once all solutions are generated." << std::endl;
        solver_thread_routine(&db);
    }


    // the main program will take care of writing
    stream.clear();
    writer_thread_routine(std::ref(stream), print_every);
    // close the stream
    stream.close();

    // join worker threads
    for(auto &thread : worker_threads)
        thread.join();

    return 0;
}
