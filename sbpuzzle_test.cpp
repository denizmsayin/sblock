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

class ManhattanHeuristic {
public:
    int operator()(const SBPuzzle &p) {
        return p.manhattan_distance_to_solution();
    }
};

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

int main() {
    auto rng = std::default_random_engine(SEED);
    vector<SBPuzzle> puzzles;
    for(int i = 0; i < N; i++) 
        puzzles.push_back(create_solvable_puzzle(H, W, rng));

    auto t1 = std::chrono::high_resolution_clock::now();
    int num_moves = 0;
    for(int i = 0; i < N; i++) {
        /*
        cout << i << endl;
        vector<Dir> moves1 = graphSearch<SBPuzzle, Dir, ManhattanHeuristic, PriorityQueue>(puzzles[i], Dir::INVALID);
        vector<Dir> moves2 = graphSearch<SBPuzzle, Dir, NoHeuristic, Queue>(puzzles[i], Dir::INVALID);
        if(!check_equality(moves1, moves2) && moves1.size() != moves2.size()) {
            cout << "Not equal!" << endl;
            cout << puzzles[i] << endl;
            cout << "M1: ";
            for(auto m : moves1)
                cout << m << " ";
            cout << endl;
            SBPuzzle p1(puzzles[i]);
            p1.apply_moves(moves1);
            cout << p1 << endl;
            cout << "M2: ";
            for(auto m : moves2)
                cout << m << " ";
            cout << endl;
            SBPuzzle p2(puzzles[i]);
            p2.apply_moves(moves2);
            cout << p2 << endl;

        }
        */
        // vector<Dir> moves = graphSearch<SBPuzzle, Dir, ManhattanHeuristic, PriorityQueue>(puzzles[i], Dir::INVALID);
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
