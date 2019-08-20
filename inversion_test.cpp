#include <iostream>

#include <random>
#include <vector>

#include "sbpuzzle.hpp"

using namespace std;

constexpr int T = 100;
constexpr unsigned SEED = 42;
constexpr unsigned SIZE = 3;

template <typename Iterator, typename URNG, typename Distr>
void insert_random_numbers(Distr &d, URNG &rng, Iterator itr, int size) {
    for(int i = 0; i < size; i++) 
        *itr++ = d(rng);
}

int main() {
    auto rng = default_random_engine(SEED);
    uniform_int_distribution<int> uni(0, 9);
    vector<int> v(SIZE);
    for(int i = 0; i < T; i++) {
        insert_random_numbers(uni, rng, v.begin(), SIZE);
        int inv1 = count_inversions(v.begin(), v.end());
        int inv2 = count_inversions_q_nomodif(v.begin(), v.end());
        if(inv1 != inv2) {
            cout << "No match! inv1: " << inv1 << ", inv2: " << inv2 << endl;
            for(const auto &x : v)
                cout << x << " ";
            cout << endl;
        }
    }
    return 0;
}
