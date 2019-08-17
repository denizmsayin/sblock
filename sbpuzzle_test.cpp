#include "search.hpp"
#include "search_queues.hpp"
#include "sbpuzzle.h"

#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <chrono>

using namespace std;
using Dir = SBPuzzle::Direction;

unsigned SEED = 42;

int N = 100;
int H = 3, W = 3;

template <class URNG>
SBPuzzle create_solvable_puzzle(int h, int w, URNG &&rng) {
    SBPuzzle puzzle(h, w);
    do {
        puzzle.shuffle(rng);
    } while(!puzzle.is_solvable());
    return puzzle;
}

class NoHeuristic {
public:
    int operator()(const SBPuzzle &p) {
        return 0;
    }
};

int main() {
    auto rng = std::default_random_engine(SEED);
    vector<SBPuzzle> puzzles;
    for(int i = 0; i < N; i++) 
        puzzles.push_back(create_solvable_puzzle(H, W, rng));

    auto t1 = std::chrono::high_resolution_clock::now();
    int num_moves = 0;
    for(int i = 0; i < N; i++) {
        vector<Dir> moves = graphSearch<SBPuzzle, Dir, NoHeuristic, Queue>(puzzles[i], Dir::INVALID);
        num_moves += moves.size();
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;

    double avg_moves = static_cast<double>(num_moves) / N;
    std::cout << "Solved " << N << " puzzles in " << fp_ms.count() / N << " milliseconds on "
              << "average with " << avg_moves << " moves on average." << endl;

    return 0;
}
