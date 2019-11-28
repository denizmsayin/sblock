#include "sblock_utils.hpp"
#include <iostream>

using namespace std;
int main() {
    int x = 0;
    SeriesTracker<int>::Options opts;
    opts.print_every = 1000000;
    opts.alpha = 0.33;
    SeriesTracker<int> tracker(&x, opts);
    for(; x < 1000000000; ++x) 
        tracker.track();
    return 0;
}
