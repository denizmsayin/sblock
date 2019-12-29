#ifndef __DPDB_HPP__
#define __DPDB_HPP__

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>
#include <cstdio>
#include <queue>

#include "pdb.hpp"
#include "sbpuzzle.hpp"
#include "sblock_utils.hpp"
#include "search2.hpp"

namespace sbpuzzle {
// Now, since our function that determine
// 'generically' over an iterator that returns booleans, we need to
// create a custom iterator class constructed over a set of tiles
// that presents a 'boolean' view of them
    namespace details {
        template <psize_t H, psize_t W>
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

        template <psize_t H, psize_t W>
        size_t calculate_table_index(const std::array<uint8_t, H*W> &tiles,
                                     const std::array<bool, H*W> group_mask,
                                     uint8_t group_size)
        {
            // first, find the combination index
            auto s = TileCombIterator<H, W>::begin(group_mask, tiles);
            auto e = TileCombIterator<H, W>::end(group_mask, tiles);
            size_t comb_i = calculate_combindex(s, e, tiles.size(), group_size);
            // then the lexicographical index of the group elements
            std::vector<uint8_t> group_tiles(group_size);
            details::tiles_fill_group<H, W>(group_mask, tiles, group_tiles.begin());
            size_t lex_i = calculate_lexindex(group_tiles.begin(), group_tiles.end());
            // calculate the total index and lookup the table
            size_t index = comb_i * factorial(group_size) + lex_i;
            return index;
        }

    }

    template <psize_t H, psize_t W>
    class DPDB : public PDB<H, W> {
    public:

        // groups is supposed to contain a group for each tile
        // e.g. for 3x3 and 2 groups : {0, 0, 0, 1, 1, 1, 2, 2, 0}
        // Note that the group of the last element (hole) is ignored,
        // and that the groups MUST be enumerated from 0 to N-1

        // function for a set of groups
        static DPDB from_file(const std::string &filename);
        static DPDB generate(const std::array<uint8_t, H*W> &o_groups);
        static void generate_and_save(const std::array<uint8_t, H*W> &o_groups, 
                                      const std::string &filename);

        void save(const std::string &filename) const;

        uint8_t lookup(const std::array<uint8_t, H*W> &tiles) const;
        uint8_t lookup(const SBPuzzleBase<H, W> &p) const;

        // both static variables are used for printing details during generation
        // they should be changed before calling generate/generate_and_save
        static SeriesTracker<size_t>::Options GEN_TRACKER_OPTS;
        static bool GEN_VERBOSE;

        // calculate the index of the given tile configuration for the given group
        // note that DPDB works with the no-hole version of SBPuzzle. extended
        // implies taking into account the hole position, which is used for 
        // the custom BFS inside _generate
    private:
        DPDB(const std::array<uint8_t, H*W> &o_groups);
       
        size_t calculate_index(int group_num, const std::array<uint8_t, H*W> &tiles) const;
        size_t calculate_extended_index(int group_num, const std::array<uint8_t, H*W> &tiles) const;

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

    template <psize_t H, psize_t W>
    SeriesTrackedValue<size_t>::Options DPDB<H, W>::GEN_TRACKER_OPTS = 
        SeriesTrackedValue<size_t>::Options{}
            .do_track(false)
            .print_every(1000000)
            .name_str("States explored");

    template <psize_t H, psize_t W>
    bool DPDB<H, W>::GEN_VERBOSE = false;

    // How are the tables laid out? 
    // -> For each group, there is a table with C(Ntiles, Ngroup) * Ngroup! entries
    // e.g. for a group with 7 elements in the 15 puzzle C(16, 7) * 7! = 57,657,600
    // -> To index each table, we first have to determine where the blocks belonging to
    // the group are, as the first part of the index (the C(Ntiles, Ngroup) part). 
    // -> For the second index we simply calculate the lexicographical index of the ordering
    // of the members of the group (the Ngroup! part)


    template <psize_t H, psize_t W>
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

    template <psize_t H, psize_t W>
    DPDB<H, W>::DPDB(const std::array<uint8_t, H*W> &o_groups) {
        init(o_groups);
    }

    // This function is necessary for easy use of the calculate_lexindex
    // function. As all tiles in a group have to be in a contiguous array
    // for their index to be calculated, this function simply fills
    // the output iterator with consecutive tiles from the same group

    template <psize_t H, psize_t W>
    class ZH {
    public:
        constexpr int operator()(const SBPuzzleNoHole<H, W> &p) {
            return 0;
        }
    };

    template <psize_t H, psize_t W>
    class MH {
    public:
        constexpr int operator()(const SBPuzzleNoHole<H, W> &p) {
            return p.manhattan_distance_to_solution();
        }
    };

    template <psize_t H, psize_t W>
    DPDB<H, W> DPDB<H, W>::from_file(const std::string &filename) {
        // Check DPDB<H, W>::save for a description of the format
        /*
        std::ifstream in_file(filename, std::ifstream::binary);
        char h = in_file.get();
        char w = in_file.get();
        */
        FILE *f = fopen(filename.c_str(), "rb");
        if(!f) throw std::runtime_error("Could not open provided PDB file: " + filename);
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

    template <psize_t H, psize_t W>
    void DPDB<H, W>::generate_and_save(
            const std::array<uint8_t, H*W> &o_groups,
            const std::string &filename) 
    {
        DPDB<H, W> db = DPDB<H, W>::generate(o_groups);
        db.save(filename);
    }

    template <psize_t H, psize_t W>
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

    template <psize_t H, psize_t W>
    DPDB<H, W> DPDB<H, W>::generate(const std::array<uint8_t, H*W> &o_groups) {
        DPDB db(o_groups);
        for(uint8_t i = 0; i < db.num_groups; ++i)
            db._generate(i);
        return db;
    }

    template <psize_t H, psize_t W>
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
        
        // the size of the visited lookup table will be +1 compared to the hole-less group
        // so (group_size + 1) times the cost lookup table
        // only a fraction will be utilized, but hopefully vector<bool> will help with space
        size_t visited_size = combination(SIZE, group_counts[i] + 1) 
                              * factorial(group_counts[i] + 1);
        
        // sadly, bitset size has to be known at compile time,
        // so we're left with vector<bool>...
        std::vector<bool> visited(visited_size, false);
        
        // a tracker for tracking the progress of generation
        // update the target of the options as the number of entries in the db
        GEN_TRACKER_OPTS.target_value(tables[i].size());
       
        if(GEN_VERBOSE)
            std::cout << "Generating database for group " 
                      << static_cast<int>(i) << "..." << std::endl;

        // a small local struct as a bfs node
        // ideally, SBPuzzleNoHole could be serialized
        struct Node {
            PuzzleType puzzle;
            uint8_t cost;
            Node(const PuzzleType &p, uint8_t c) : puzzle(p), cost(c) {}
        };

        // the actual modified BFS implementation
        std::vector<uint8_t> &table = tables[i];
        PuzzleType start_state(tiles, in_groups[i]);
        std::queue<Node> q;
        q.emplace(start_state, 0);
        size_t start_ex_index = calculate_extended_index(i, start_state.get_tiles());
        visited[start_ex_index] = true;
        {
            SeriesTrackedValue<size_t> ex_states(0, GEN_TRACKER_OPTS);
            while(!q.empty()) {
                Node node = q.front(); q.pop(); // get the next node
                const auto &p = node.puzzle;
                if(node.cost == DIST_MAX)
                    std::cerr << "Warning: cost overflow in DPDB BFS" << std::endl;
                // update cost in table if necessary (with reduced index)
                size_t index = calculate_index(i, p.get_tiles());
                if(table[index] == DIST_MAX) // count explored states only on first find
                    ++ex_states;
                if(node.cost < table[index])
                    table[index] = node.cost;
                // generate neighboring states
                for(const auto &action : p.template action_generator<ActionType>()) {
                    PuzzleType new_p = p; 
                    uint8_t new_cost = node.cost + new_p.apply_action(action);
                    size_t new_ex_index = calculate_extended_index(i, new_p.get_tiles());
                    if(!visited[new_ex_index]) {
                        // can already be marked as visited before actually
                        // expanding. Why? As soon as a node is added to the open
                        // list, the shortest path to it has been found for BFS
                        visited[new_ex_index] = true;
                        q.emplace(new_p, new_cost);
                    }
                }
            }
        }

        if(GEN_VERBOSE) {
            // count the number of false values in visited, because I wonder!
            // somewhere between table_size and table_size * (group_size + 1)
            size_t visited_count = std::count(visited.begin(), visited.end(), true);
            std::cout << "Visited table usage: " 
                      << visited_count << '/' << visited.size() << std::endl;
        }

        /*
        search2::BreadthFirstIterator<PuzzleType, ActionType> itr(start_state);
        while(!itr.done()) {
            auto node = itr.next();
            size_t index = node.puzzle.determine_index(i, *this);
            if(tables[i][index] == DIST_MAX)
                std::cout << node.puzzle << std::endl;
            size_t ex_index = node.puzzle.determine_extended_index(i, *this);
            if(!visited[ex_index])
                std::cout << node.puzzle << std::endl;
        }
        */
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

        /*
        PuzzleType start_puzzle(tiles, in_groups[i]);
        search2::BreadthFirstIterator<PuzzleType, ActionType> itr(start_puzzle);
        while(!itr.done()) {
            auto node = itr.next();
            const auto &p = node.puzzle;
            // find the table index of the node's state
            auto index = p.determine_index(i, *this);
            if(node.path_cost < tables[i][index])
                tables[i][index] = node.path_cost;

        }
        */
    }

    template <psize_t H, psize_t W>
    size_t DPDB<H, W>::calculate_index(int i, 
                                       const std::array<uint8_t, H*W> &o_tiles) const
    {
        return details::calculate_table_index<H, W>(o_tiles, in_groups[i], group_counts[i]);
    }

    template <psize_t H, psize_t W>
    size_t DPDB<H, W>::calculate_extended_index(int i, 
                                                const std::array<uint8_t, H*W> &o_tiles) const 
    {
        // if extended, we need to account for the hole after collapsing it
        std::array<uint8_t, H*W> tiles = o_tiles; // copying output tiles to keep constness
        std::array<bool, H*W> group_mask = in_groups[i];
        uint8_t group_size = group_counts[i];
        // collapse the hole and add it to the group
        details::tiles_collapse_hole<H, W>(tiles);
        group_mask[details::HOLE<H, W>] = true;
        ++group_size;
        return details::calculate_table_index<H, W>(tiles, group_mask, group_size);
    }

    /*
    template <psize_t H, psize_t W>
    int DPDB<H, W>::lookup(const SBPuzzleNoHole<H, W> &p) const {
        return lookup(p.get_tiles());
    }
    */

    template <psize_t H, psize_t W>
    uint8_t DPDB<H, W>::lookup(const std::array<uint8_t, H*W> &tiles) const {
        uint8_t total = 0;
        // for each group
        for(uint8_t i = 0; i < num_groups; ++i) {
            size_t index = calculate_index(i, tiles);
            total += tables[i][index];
        }
        return total;
    }

}

#endif
