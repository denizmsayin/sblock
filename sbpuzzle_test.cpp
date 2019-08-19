#include "search.hpp"
#include "search_queues.hpp"
#include "sbpuzzle.hpp"

#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <chrono>

unsigned SEED = 42;

constexpr int N = 1;
constexpr int H = 4, W = 4;

using namespace std;
using Dir = Direction;

template <int H, int W, class URNG>
SBPuzzle<H, W> create_solvable_puzzle(URNG &&rng) {
    SBPuzzle<H, W> puzzle;
    do {
        puzzle.shuffle(rng);
    } while(!puzzle.is_solvable());
    return puzzle;
}

template <int H, int W>
class MisplacedTilesHeuristic {
public:
    int operator()(const SBPuzzle<H, W> &p) {
        return p.num_misplaced_tiles();
    }
};

template <int H, int W>
class ManhattanHeuristic {
public:
    int operator()(const SBPuzzle<H, W> &p) {
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
    auto rng = default_random_engine(SEED);
    vector<SBPuzzle<H, W>> puzzles;
    for(int i = 0; i < N; i++) 
        puzzles.push_back(create_solvable_puzzle<H, W>(rng));

    auto t1 = chrono::high_resolution_clock::now();
    int num_moves = 0;
    for(int i = 0; i < N; i++) {
        /* 
        cout << i << endl;
        vector<Dir> moves1 = a_star_search<SBPuzzle, Dir, ManhattanHeuristic>(puzzles[i], Dir::INVALID);
        vector<Dir> moves2 = bidirectional_bfs<SBPuzzle, Dir>(puzzles[i], Dir::INVALID);
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
        // vector<Dir> moves = breadth_first_search<SBPuzzle<H, W>, Dir>(puzzles[i], Dir::INVALID);
        // vector<Dir> moves = a_star_search<SBPuzzle<H, W>, Dir, ManhattanHeuristic<H, W>>(puzzles[i], Dir::INVALID);
        // vector<Dir> moves = bidirectional_bfs<SBPuzzle<H, W>, Dir>(puzzles[i], Dir::INVALID);
        vector<Dir> moves = iterative_deepening_a_star<SBPuzzle<H, W>, Dir, ManhattanHeuristic<H, W>>(puzzles[i], Dir::INVALID);
        /*
        SBPuzzle p(puzzles[i]);
        p.apply_moves(moves);
        cout << p << endl;
        */
        num_moves += moves.size();
        cout << i << endl;
    }
    auto t2 = chrono::high_resolution_clock::now();

    chrono::duration<double, std::milli> fp_ms = t2 - t1;

    double avg_moves = static_cast<double>(num_moves) / N;
    cout << "Solved " << N << " puzzles in " << fp_ms.count() / N << " milliseconds on "
         << "average with " << avg_moves << " moves on average." << endl;

    return 0;
}
