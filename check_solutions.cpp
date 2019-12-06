#include <fstream>
#include <array>
#include <cstdint>

#include "search2.hpp"
#include "sbpuzzle.hpp"
#include "crdpdb.hpp"

#ifndef __H
#define __H 3
#endif
#ifndef __W
#define __W 3
#endif

// program constants
constexpr uint8_t H = __H, W = __W;

// typedefs for practicality & readability
typedef sbpuzzle::SBPuzzleWHole<H, W> Puzzle;
typedef sbpuzzle::TileSwapAction Action;
typedef sbpuzzle::PDB<H, W> PDB;
typedef sbpuzzle::CRDPDB<H, W> CRDPDB;

int main(int argc, char *argv[]) {
    using namespace std;
    if(argc != 3) {
        cout << "Usage: ./check_solutions db_file solution_file" << endl;
        return 0;
    }

    auto db = CRDPDB::from_file(argv[1]);

    std::array<uint8_t, H*W> tiles;
    fstream f(argv[2], fstream::in | fstream::binary);
    while(f) {
        f.read(reinterpret_cast<char *>(&tiles[0]), sizeof(tiles[0]) * tiles.size());
        uint8_t read_cost = f.get();
        Puzzle p(tiles);
        auto r = search2::a_star_search<Puzzle, Action>(p, [&](const Puzzle &p) { 
                return p.lookup_cost(&db); 
        });
        if(read_cost != r.cost) {
            cout << "Cost mismatch!" << endl
                 << p << endl
                 << "Read cost: " << read_cost << endl
                 << "Calc cost: " << r.cost << endl;
            return 0;
        }
    }
    
    cout << "Check successful." << endl;
    return 0;
}
