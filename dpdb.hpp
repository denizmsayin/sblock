#ifndef __DPDB_HPP__
#define __DPDB_HPP__

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>
#include <cstdio>
#include <queue>

#include "sbpuzzle.hpp"
#include "sblock_utils.hpp"
#include "search2.hpp"

namespace sbpuzzle {
// Now, since our function that determine
// 'generically' over an iterator that returns booleans, we need to
// create a custom iterator class constructed over a set of tiles
// that presents a 'boolean' view of them
    namespace details {
        template <details::psize_t H, details::psize_t W>
        class TileCombIterator {
        public:
            // IMPORTANT: the iterator does not perform a copy of the input
            // tiles and simply uses the raw interpreted pointer. This is
            // why the tiles shouldn't change during the lifetime of a single
            // TileCombIterator
            // inp_tiles is simply the tile configuration as per sbpuzzle.hpp
            // in_group is an array, which shows true if a given tile is in the group
            // e.g. if we have a 3x3 puzzle and out group is {0 2 3 5}
            // in_group = [t f t t f t f f f]
            // Also, equality checks only compare the index, so it's up to the
            // user to ensure that tiles are the same
            static TileCombIterator begin(const std::array<bool, H*W> &in_group, 
                                          const std::array<uint8_t, H*W> &tiles) 
            {
                return TileCombIterator(&in_group.front(), &tiles.front());
            }
            
            static TileCombIterator end(const std::array<bool, H*W> &in_group, 
                                        const std::array<uint8_t, H*W> &tiles) 
            {
                return TileCombIterator(&in_group.back(), &tiles.back());
            }

                /*
                using std::cout;
                using std::endl;
                std::cout << "Combitrcreated" << std::endl;
                for(auto i = 0; i < H*W; ++i)
                    cout << static_cast<int>(inp_tiles[i]) << " ";
                cout << endl;
                for(auto i = 0; i < H*W; ++i)
                    cout << inp_in_group[i] << " ";
                cout << endl;
                cout << static_cast<int>(xv) << endl;
                for(auto i = 0; i < H*W; ++i)
                    cout << (inp_tiles[i] != xv && inp_in_group[inp_tiles[i]]) << " ";
                cout << endl;
                */

            TileCombIterator(const TileCombIterator &other) = default;
            TileCombIterator(TileCombIterator &&other) = default;
            TileCombIterator& operator=(const TileCombIterator &other) = default;
            TileCombIterator& operator=(TileCombIterator &&other) = default;

            const TileCombIterator &operator++() { // prefix ++
                ++tiles;
                return *this;
            }

            bool operator*() const {
                return *tiles != details::_X && in_group[*tiles];
            }

            bool operator==(const TileCombIterator &other) const {
                return tiles == other.tiles;
            }

            bool operator!=(const TileCombIterator &other) const {
                return tiles != other.tiles;
            }

        private:
            TileCombIterator(const bool *o_in_group, 
                             const uint8_t *o_tiles) 
                : in_group(o_in_group), tiles(o_tiles)
            {}

            // TODO: make this even better by somehow squeezing into 8 bytes
            const bool *in_group;
            const uint8_t *tiles;
        };
    }

    template <details::psize_t H, details::psize_t W>
    class DPDB {
    public:

        // groups is supposed to contain a group for each tile
        // e.g. for 3x3 and 2 groups : {0, 0, 0, 1, 1, 1, 2, 2, 0}
        // Note that the group of the last element (hole) is ignored,
        // and that the groups MUST be enumerated from 0 to N-1
        template <typename FIterator>
        DPDB(const std::array<uint8_t, H*W> &o_groups, FIterator filenames_begin, FIterator filenames_end);

        // function for a set of groups
        static DPDB from_file(const std::string &filename);
        static DPDB generate(const std::array<uint8_t, H*W> &o_groups);
        static void generate_and_save(const std::array<uint8_t, H*W> &o_groups, 
                                      const std::string &filename);
        
        void save(const std::string &filename) const;

        int lookup(const std::array<uint8_t, H*W> &tiles) const;

        // calculate the index of the given tile configuration for the given group
        // note that DPDB works with the no-hole version of SBPuzzle. extended
        // implies taking into account the hole position, which is used for 
        // the custom BFS inside _generate
        size_t calculate_table_index(int group_num, const std::array<uint8_t, H*W> &tiles, bool extended=false) const;
    private:
        DPDB(const std::array<uint8_t, H*W> &o_groups);
        
        void init(const std::array<uint8_t, H*W> &o_groups);
        
        template <class OutputIterator>
        void fill_group(int group_num, 
                        const std::array<uint8_t, H*W> &tiles, 
                        OutputIterator itr) const;
        
        // function for a single group, with a dpdb object prepared
        void _generate(uint8_t group_num);

        std::vector<std::vector<uint8_t>> tables;
        std::array<uint8_t, H*W> groups;
        uint8_t num_groups;
        std::vector<uint8_t> group_counts;
        std::vector<std::array<bool, H*W>> in_groups;

        constexpr static int SIZE = H * W;
        constexpr static uint8_t DIST_MAX = 255;
    };

    // How are the tables laid out? 
    // -> For each group, there is a table with C(Ntiles, Ngroup) * Ngroup! entries
    // e.g. for a group with 7 elements in the 15 puzzle C(16, 7) * 7! = 57,657,600
    // -> To index each table, we first have to determine where the blocks belonging to
    // the group are, as the first part of the index (the C(Ntiles, Ngroup) part). 
    // -> For the second index we simply calculate the lexicographical index of the ordering
    // of the members of the group (the Ngroup! part)


    template <details::psize_t H, details::psize_t W>
    void DPDB<H, W>::init(const std::array<uint8_t, H*W> &o_groups) 
    {
        // copy the group number of each tile
        std::copy(o_groups.begin(), o_groups.end(), groups.begin());
        // find the number of groups, assuming the largest element is it
        // use a custom comparator to ignore X values
        num_groups = 1 + *std::max_element(groups.begin(), groups.end(), [](auto x, auto y) {
                x = (x == DONT_CARE) ? 0 : x;
                y = (y == DONT_CARE) ? 0 : y;
                return x < y;
        });
        // count each group's elements and mark them
        // also allocate empty tables
        group_counts = std::vector<uint8_t>(num_groups, 0);
        in_groups = std::vector<std::array<bool, SIZE>>(num_groups);
        for(uint8_t i = 0; i < num_groups; ++i)
            in_groups[i].fill(false);
        for(int i = 0; i < SIZE; ++i) {
            if(groups[i] != details::_X) {
                group_counts[groups[i]]++;
                in_groups[groups[i]][i] = true; // using 1/0 since vbool is an uint type
            }
        } // NOTE: the hole is not in any group
        // initialize empty tables
        tables = std::vector<std::vector<uint8_t>>(num_groups);
        for(uint8_t i = 0; i < num_groups; ++i) {
            size_t db_size = combination(SIZE, group_counts[i]) * factorial(group_counts[i]);
            tables[i] = std::vector<uint8_t>(db_size, DIST_MAX);
            // set the initial distances to max value
        }
    }

    template <details::psize_t H, details::psize_t W>
    DPDB<H, W>::DPDB(const std::array<uint8_t, H*W> &o_groups) {
        init(o_groups);
    }

    template <details::psize_t H, details::psize_t W>
    template <typename FIterator>
    DPDB<H, W>::DPDB(
            const std::array<uint8_t, H*W> &o_groups,
            FIterator filenames_begin, 
            FIterator filenames_end) 
    {
        init(o_groups);
        FIterator fname = filenames_begin;
        for(uint8_t i = 0; i < num_groups; ++i) 
            read_byte_array(&(tables[i][0]), tables[i].size(), *fname++);
    }

    // This function is necessary for easy use of the calculate_lexindex
    // function. As all tiles in a group have to be in a contiguous array
    // for their index to be calculated, this function simply fills
    // the output iterator with consecutive tiles from the same group

    template <details::psize_t H, details::psize_t W>
    class ZH {
    public:
        constexpr int operator()(const SBPuzzleNoHole<H, W> &p) {
            return 0;
        }
    };

    template <details::psize_t H, details::psize_t W>
    class MH {
    public:
        constexpr int operator()(const SBPuzzleNoHole<H, W> &p) {
            return p.manhattan_distance_to_solution();
        }
    };

    template <details::psize_t H, details::psize_t W>
    DPDB<H, W> DPDB<H, W>::from_file(const std::string &filename) {
        // Check DPDB<H, W>::save for a description of the format
        /*
        std::ifstream in_file(filename, std::ifstream::binary);
        char h = in_file.get();
        char w = in_file.get();
        */
        FILE *f = fopen(filename.c_str(), "rb");
        int h = fgetc(f);
        int w = fgetc(f);
        if(h != H || w != W)
            throw std::invalid_argument("Stored DPDB dimensions do not match template function");
        std::array<uint8_t, H*W> groups;
        ssize_t s = fread(&groups[0], sizeof(groups[0]), H*W, f);
        if(s != H*W)
            throw std::runtime_error("Failed to read enough bytes");
        // TODO: ifstream fails to read the 255 values, which is why
        // I switched to cstdio for this function. Would be good to fix.
        // in_file.get(reinterpret_cast<char *>(&groups[0]), H*W);
        DPDB<H, W> db(groups);
        for(uint8_t i = 0; i < db.num_groups; ++i) {
            // in_file.get(reinterpret_cast<char *>(&(db.tables[i][0])), db.table_sizes[i]);
            s = fread(&(db.tables[i][0]), sizeof(db.tables[i][0]), db.tables[i].size(), f);
            if(s != static_cast<ssize_t>(db.tables[i].size()))
                throw std::runtime_error("Failed to read enough bytes");
        }
        // in_file.close();
        fclose(f);
        return db;
    }

    template <details::psize_t H, details::psize_t W>
    void DPDB<H, W>::generate_and_save(
            const std::array<uint8_t, H*W> &o_groups,
            const std::string &filename) 
    {
        DPDB<H, W> db = DPDB<H, W>::generate(o_groups);
        db.save(filename);
    }

    template <details::psize_t H, details::psize_t W>
    void DPDB<H, W>::save(const std::string &filename) const {
        // A DPDB file is a binary file with the following format:
        // first 2 bytes: H & W
        // following H*W bytes: group specification
        // remaining bytes: table for group 0, table for group 1, ...
        std::ofstream out_file(filename, std::ofstream::binary);
        out_file.put(H);
        out_file.put(W);
        out_file.write(reinterpret_cast<const char *>(&groups[0]), H*W);
        for(uint8_t i = 0; i < num_groups; ++i)
            out_file.write(reinterpret_cast<const char *>(&tables[i][0]), tables[i].size());
        out_file.close();
    }

    template <details::psize_t H, details::psize_t W>
    DPDB<H, W> DPDB<H, W>::generate(const std::array<uint8_t, H*W> &o_groups) {
        DPDB db(o_groups);
        for(uint8_t i = 0; i < db.num_groups; ++i)
            db._generate(i);
        return db;
    }

    template <details::psize_t H, details::psize_t W>
    void DPDB<H, W>::_generate(uint8_t i) 
    {
        typedef SBPuzzleNoHole<H, W> PuzzleType;
        typedef TileSwapAction ActionType;
        // we simply need to apply bfs, but only add a cost when a tile
        // from the group is moved, otherwise the moves do not cost anything
        std::array<uint8_t, SIZE> tiles;
        std::iota(tiles.begin(), tiles.end(), 0);
        // TODO: extend this so it can deal with partitions where the group
        // does not split the puzzle into multiple parts. e.g.
        // -------------
        // | X | X | X | valid group, does not split the puzzle, single
        // ------------- connected X blob
        // | 3 | 4 | X |
        // -------------
        // | 6 | 7 | X |
        // -------------
        // -------------
        // | X | X | 2 | invalid group, splits the puzzle into two X blobs
        // -------------
        // | 3 | 4 | 5 |
        // -------------
        // | X | X | X |
        // -------------
        // This is troublesome because in the second case, we need two different
        // starting states with the hole being in each blob, and combine the results
        // we obtain from either of them.
        //
        // PS: Scratch that, it probably works now


        // Instead of using search2::BFS, I decided to implement this BFS right 
        // here with a few modifications. One of the main things is making use
        // of a lookup table for indexing, rather than using a hash set which
        // takes up a lot of space due to the puzzle state not being compressed.
        // However, do note that it is not possible to store results in the actual
        // table since the hole position matters for the search even if not for the
        // database. 
        //
        // Encoding a representation containing an arbitrary amount of X & H values
        // seems difficult. Thus, I will implement a simple if imperfect trick. For
        // each state, the hole region will be collapsed into a single hole at the
        // final possible position. Then, the indexing will also take into account
        // the position of the hole. This will actually consume more memory than
        // necessary because not all hole positions are possible, but with a bitset
        // this should still use way less space than a hash table storing nodes.
        // e.g. state 0 1 2 3 H H H H H -> 0 1 2 3 X X X X H
        //      but the table also has spots for 0 1 2 3 X X X H X
        //                                       0 1 2 3 X X H X X etc.
        // TODO: make this better by somehow finding a way to remove redundant positions
        
        
        // a tracker for tracking the progress of generation
        #ifdef TRACK_DPDB
        size_t sc = 0;
        SeriesTracker<size_t>::Options opts;
        opts.print_every = 10000000;
        SeriesTracker<size_t> t(&sc, opts);
        #endif

//        // a small local struct as a bfs node
//        // ideally, SBPuzzleNoHole could be serialized
//        struct Node {
//            SBPuzzleNoHole<H, W> puzzle;
//            uint8_t cost;
//            Node(const SBPuzzleNoHole<H, W> &p, uint8_t c) : puzzle(p), cost(c) {}
//        };
//
//        // the actual modified BFS implementation
//        using TSA = sbpuzzle::TileSwapAction;
//        std::vector<uint8_t> &table = tables[i];
//        for(size_t i = 0; i < table.size(); ++i)
//            if(table[i] != DIST_MAX)
//                std::cout << "entry at " << i << " is " << table[i] << std::endl;
//        SBPuzzleNoHole<H, W> start_state(tiles, in_groups[i]);
//        std::queue<Node> q;
//        q.emplace(start_state, 0);
//        while(!q.empty()) {
//            auto node = q.front(); q.pop(); // get the next node
//            const auto &p = node.puzzle;
//            if(node.cost == DIST_MAX)
//                std::cerr << "Warning: cost overflow in DPDB BFS" << std::endl;
//            // find the table index of the node's state
//            auto index = p.determine_index(i, *this);
//            // insert the path cost only if it is less than one found so far
//            // this is necessary because the hole position is not accounted for,
//            // and multiple states & paths can lead to the same index
//            // not visited / lower cost found
//            if(node.cost < table[index]) {
//                table[index] = node.cost; // essentially, mark as visited
//                for(auto action : p.template possible_actions<TSA>()) {
//                    std::cout << '(' << ((int)action.tpos) << ", " << ((int)action.hpos) << ") ";
//                    SBPuzzleNoHole<H, W> new_p = p; 
//                    uint8_t new_cost = node.cost + new_p.apply_action(action);
//                    auto new_index = new_p.determine_index(i, *this);
//                    if(new_cost < table[new_index])
//                        q.emplace(new_p, new_cost);
//                    else 
//                        std::cout << "NC: " << ((int)new_cost) << ", TC: " << ((int)table[new_index]) << ", I: " << new_index << std::endl << new_p;
//                    std::cout << std::endl;
//                }
//            }
//            std::cout << std::endl;
//            std::cout << node.puzzle << std::endl;
//            std::cout << static_cast<int>(node.cost) << std::endl;
//            std::cout << index << std::endl;
//            std::cout << "qsize: " << q.size() << std::endl;
//            std::cout << "*************************" << std::endl;
            
            // I believe BFS fails for some problem instances. Therefore,
            // for each problem I want to solve it using A* and compare the path cost
            /*
            SBPuzzleNoHole<H, W> p2 = node.puzzle;
            int cost = search2::a_star_search<SBPuzzleNoHole<H, W>, EA, MH<H, W>>(p2);
            for(auto a : p2.template possible_actions<EA>())
                std::cout << static_cast<int>(a.tpos) << "," << static_cast<int>(a.hpos) << " ";
            std::cout << std::endl;
            if(cost != node.path_cost) {
                std::cout << "Found cost: " << node.path_cost << ", A*: " << cost << std::endl;
            }
            std::cout << "----------------------" << std::endl;
            */

        PuzzleType start_puzzle(tiles, in_groups[i]);
        search2::BreadthFirstIterator<PuzzleType, ActionType> itr(start_puzzle);
        while(!itr.done()) {
            auto node = itr.next();
            const auto &p = node.puzzle;
            // find the table index of the node's state
            auto index = p.determine_index(i, *this);
            if(node.path_cost < tables[i][index])
                tables[i][index] = node.path_cost;

            #ifdef TRACK_DPDB
            sc++;
            t.track();
            #endif
        }
    }

    template <details::psize_t H, details::psize_t W>
    size_t DPDB<H, W>::calculate_table_index(int i, 
                                             const std::array<uint8_t, H*W> &o_tiles,
                                             bool extended) const 
    {
        // if extended, we need to account for the hole after collapsing it
        std::array<uint8_t, H*W> tiles = o_tiles; // copying output tiles to keep constness
        std::array<bool, H*W> group_mask = in_groups[i];
        uint8_t group_size = group_counts[i];
        if(extended) {
            // collapse the hole and add it to the group
            details::tiles_collapse_hole<H, W>(tiles);
            group_mask[details::HOLE<H, W>] = true;
            ++group_size;
        }
        // first, find the combination index
        auto s = details::TileCombIterator<H, W>::begin(group_mask, tiles);
        auto e = details::TileCombIterator<H, W>::end(group_mask, tiles);
        size_t comb_i = calculate_combindex(s, e, SIZE, group_size);
        // then the lexicographical index of the group elements
        std::vector<uint8_t> group_tiles(group_size);
        details::tiles_fill_group<H, W>(group_mask, tiles, group_tiles.begin());
        /*
        std::cout << "Comb view: ";
        for(auto x = s; x != e; ++x)
            std::cout << *x << " ";
        std::cout << std::endl;
        std::cout << "Lex view: ";
        for(int i = 0; i < group_size; ++i)
            std::cout << static_cast<int>(group_tiles[i]) << " ";
        std::cout << std::endl;
        */
        size_t lex_i = calculate_lexindex(group_tiles.begin(), group_tiles.end());
        // calculate the total index and lookup the table
        return comb_i * factorial(group_size) + lex_i;
    }

    /*
    template <details::psize_t H, details::psize_t W>
    int DPDB<H, W>::lookup(const SBPuzzleNoHole<H, W> &p) const {
        return lookup(p.get_tiles());
    }
    */

    template <details::psize_t H, details::psize_t W>
    int DPDB<H, W>::lookup(const std::array<uint8_t, H*W> &tiles) const {
        int total = 0;
        // for each group
        for(uint8_t i = 0; i < num_groups; ++i) {
            size_t index = calculate_table_index(i, tiles);
            total += tables[i][index];
        }
        return total;
    }
}

#endif
