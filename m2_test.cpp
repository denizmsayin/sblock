#include <iostream>

#include "pdb.hpp"
#include "search2.hpp"
#include "sbpuzzle.hpp"
#include "reflectdpdb.hpp"
#include "crdpdb.hpp"

using namespace std;
using sbpuzzle::SBPuzzle;

constexpr uint8_t H = 3, W = 3;

int main() {
    using sbpuzzle::PDB;
    using sbpuzzle::DPDB;
    using sbpuzzle::ReflectDPDB;
    using sbpuzzle::CRDPDB;
    /*
    bool mask[] = {true, true, true, true, false, false, false, false, false};
    uint8_t tiles1[] = {0, 7, 2, 3, 1, 4, 5, 6, 8};
    SBPuzzle<3, 3> p(tiles1, mask);
    cout << p << endl;
    cout << p.goal_state() << endl;
    int cost = search2::breadth_first_search<SBPuzzle<3, 3>, sbpuzzle::TileSwapAction>(p);
    cout << cost << endl;
    for(auto a : p.possible_actions<sbpuzzle::TileSwapAction>()) 
        cout << '(' << ((int) a.tpos) << ", " << ((int) a.hpos) << ") ";
    cout << endl;
    */
    // std::array<uint8_t, H*W> arr {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    /*
    std::array<uint8_t, H*W> g {0, 0, 0, 1, 1, 0, 1, 1, sbpuzzle::DONT_CARE};
    DPDB<H, W> db = DPDB<H, W>::generate(g);
    ReflectDPDB<H, W> rdb(db);
    PDB<H, W> *mdb = &db, *refdb = &rdb;
    std::array<uint8_t, H*W> arr {0, 5, 4, 1, 7, 8, 3, 2, 6}; // {0, 8, 5, 4, 9, 1, 10, 11,  6, 14, 2, 13, 7, 12, 15, 3};
    sbpuzzle::details::tiles_stream<H, W>(std::cout, arr) << std::endl;
    std::cout << static_cast<int>(mdb->lookup(arr)) << std::endl;
    std::cout << static_cast<int>(refdb->lookup(arr)) << std::endl;
    std::array<uint8_t, H*W> refl;
    sbpuzzle::details::tiles_reflect<H, W>(arr, refl);
    sbpuzzle::details::tiles_stream<H, W>(std::cout, refl) << std::endl;
    std::cout << static_cast<int>(mdb->lookup(refl)) << std::endl;
    std::cout << static_cast<int>(refdb->lookup(refl)) << std::endl;
    std::array<uint8_t, H*W> refl2;
    sbpuzzle::details::tiles_reflect<H, W>(refl, refl2);
    sbpuzzle::details::tiles_stream<H, W>(std::cout, refl2) << std::endl;
    */
    std::cout << "sizeof(DPDB)=" << sizeof(DPDB<H, W>) << std::endl;
    std::cout << "sizeof(ReflectDPDB)=" << sizeof(ReflectDPDB<H, W>) << std::endl;
    std::cout << "sizeof(CRDPDB)=" << sizeof(CRDPDB<H, W>) << std::endl;
    return 0;
}
