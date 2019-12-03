#ifndef __DPDB_HPP__
#define __DPDB_HPP__

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <string>
#include <cstring>

#include "sbpuzzle.hpp"
#include "sblock_utils.hpp"
#include "search2.hpp"

namespace sbpuzzle {
// Now, since our function that determine
// 'generically' over an iterator that returns booleans, we need to
// create a custom iterator class constructed over a set of tiles
// that presents a 'boolean' view of them
    namespace details {
        template <int H, int W>
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
            TileCombIterator(const bool *inp_in_group, 
                             const uint8_t *inp_tiles, 
                             size_t index, uint8_t x) : 
                in_group(inp_in_group), tiles(inp_tiles), i(index), xv(x) 
            {
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
            }

            TileCombIterator(const TileCombIterator &other) = default;
            TileCombIterator(TileCombIterator &&other) = default;
            TileCombIterator& operator=(const TileCombIterator &other) = default;
            TileCombIterator& operator=(TileCombIterator &&other) = default;

            const TileCombIterator &operator++() { // prefix ++
                i++;
                return *this;
            }

            bool operator*() const {
                return tiles[i] != xv && in_group[tiles[i]];
            }

            bool operator==(const TileCombIterator &other) const {
                return i == other.i;
            }

            bool operator!=(const TileCombIterator &other) const {
                return i != other.i;
            }

        private:
            const bool *in_group;
            const uint8_t *tiles;
            size_t i;
            uint8_t xv; // blank value, 255
        };
    }

    template <int H, int W>
    class DPDB {
    public:
        DPDB() = default;

        // groups is supposed to contain a group for each tile
        // e.g. for 3x3 and 2 groups : {0, 0, 0, 1, 1, 1, 2, 2, 0}
        // Note that the group of the last element (hole) is ignored,
        // and that the groups MUST be enumerated from 0 to N-1
        template <typename GIterator>
        DPDB(GIterator groups_begin, GIterator groups_end);
        template <typename GIterator, typename FIterator>
        DPDB(GIterator groups_begin, GIterator groups_end, FIterator filenames_begin, FIterator filenames_end);
        ~DPDB();

        // function for a set of groups
        template <typename GIterator, typename FIterator>
        static void generate_and_save(GIterator groups_begin, GIterator groups_end, FIterator filenames_begin, FIterator filenames_end);
        
        // function for a single group, with a dpdb object prepared
        template <typename GIterator>
        void _generate_and_save(int group_num, const char *filename);

        int lookup(const SBPuzzle<H, W> &p) const;

        int lookup(const uint8_t *tiles) const;
    //private:
        uint8_t **tables;
        uint8_t groups[H*W];
        uint8_t num_groups;
        uint8_t *group_counts;
        size_t *table_sizes;
        bool **in_groups;

        constexpr static int SIZE = H * W;
        

        details::TileCombIterator<H, W> boolview_begin(int group_num, const uint8_t *tiles) const 
        {
            return details::TileCombIterator<H, W>(in_groups[group_num], tiles, 0, sbpuzzle::details::_X);
        }

        details::TileCombIterator<H, W> boolview_end(int group_num, const uint8_t *tiles) const 
        {
            return details::TileCombIterator<H, W>(in_groups[group_num], tiles, SIZE, sbpuzzle::details::_X);
        }

        template <class OutputIterator>
        void fill_group(int group_num, const uint8_t *tiles, OutputIterator itr) const;
        
        template <typename GIterator>
        void init(GIterator groups_begin, GIterator groups_end);

        size_t calculate_table_index(int group_num, const uint8_t *tiles) const;
    };

    // How are the tables laid out? 
    // -> For each group, there is a table with C(Ntiles, Ngroup) * Ngroup! entries
    // e.g. for a group with 7 elements in the 15 puzzle C(16, 7) * 7! = 57,657,600
    // -> To index each table, we first have to determine where the blocks belonging to
    // the group are, as the first part of the index (the C(Ntiles, Ngroup) part). 
    // -> For the second index we simply calculate the lexicographical index of the ordering
    // of the members of the group (the Ngroup! part)

    template <int H, int W>
    template <typename GIterator>
    void DPDB<H, W>::init(GIterator groups_begin, GIterator groups_end) {
    // copy the group number of each tile
        std::copy(groups_begin, groups_end, groups);
        // find the number of groups, assuming the largest element is it
        num_groups = *std::max_element(groups, groups + SIZE - 1) + 1;
        // count each group's elements and mark them
        // also allocate empty tables
        group_counts = new uint8_t[num_groups]();
        in_groups = new bool *[num_groups];
        for(uint8_t i = 0; i < num_groups; ++i)
            in_groups[i] = new bool [SIZE]();
        for(int i = 0; i < SIZE; ++i) {
            if(groups[i] != details::_X) {
                group_counts[groups[i]]++;
                in_groups[groups[i]][i] = true;
            }
        } // NOTE: the hole is not in any group
        // initialize empty tables
        tables = new uint8_t *[num_groups];
        table_sizes = new size_t[num_groups];
        for(uint8_t i = 0; i < num_groups; ++i) {
            size_t db_size = combination(SIZE, group_counts[i]) * factorial(group_counts[i]);
            table_sizes[i] = db_size;
            tables[i] = new uint8_t[db_size];
            memset(tables[i], 255, db_size); // set the distances to max value
        }
    }

    template <int H, int W>
    template <typename GIterator>
    DPDB<H, W>::DPDB(GIterator groups_begin, GIterator groups_end) {
        init(groups_begin, groups_end);
    }

    template <int H, int W>
    template <typename GIterator, typename FIterator>
    DPDB<H, W>::DPDB(
            GIterator groups_begin, 
            GIterator groups_end, 
            FIterator filenames_begin, 
            FIterator filenames_end) 
    {
        init(groups_begin, groups_end);
        FIterator fname = filenames_begin;
        for(uint8_t i = 0; i < num_groups; ++i) 
            read_byte_array(tables[i], table_sizes[i], *fname++);
    }

    template <int H, int W>
    DPDB<H, W>::~DPDB() {
        delete[] group_counts;
        for(uint8_t i = 0; i < num_groups; ++i) {
            delete[] tables[i];
            delete[] in_groups[i];
        }
        delete[] tables;
        delete[] table_sizes;
        delete[] in_groups;
    }

    // This function is necessary for easy use of the calculate_lexindex
    // function. As all tiles in a group have to be in a contiguous array
    // for their index to be calculated, this function simply fills
    // the output iterator with consecutive tiles from the same group
    template <int H, int W>
    template <class OutputIterator>
    void DPDB<H, W>::fill_group(int group_num, const uint8_t *tiles, OutputIterator itr) const {
        for(int i = 0; i < SIZE; ++i)
            if(tiles[i] != sbpuzzle::details::_X && in_groups[group_num][tiles[i]])
                *itr++ = tiles[i];
    }

    template <int H, int W>
    template <typename GIterator, typename FIterator>
    void DPDB<H, W>::generate_and_save(
            GIterator groups_begin, 
            GIterator groups_end,
            FIterator filenames_begin,
            FIterator filenames_end) 
    {
        DPDB<H, W> db(groups_begin, groups_end);
        FIterator fname = filenames_begin;
        for(uint8_t i = 0; i < db.num_groups; ++i) 
            db._generate_and_save<GIterator>(i, *fname++);
    }

    template <int H, int W>
    class ZH {
    public:
        constexpr int operator()(const SBPuzzle<H, W> &p) {
            return 0;
        }
    };

    template <int H, int W>
    class MH {
    public:
        constexpr int operator()(const SBPuzzle<H, W> &p) {
            return p.manhattan_distance_to_solution();
        }
    };

    template <int H, int W>
    template <typename GIterator>
    void DPDB<H, W>::_generate_and_save(int i, const char *filename) 
    {
        using EA = sbpuzzle::TileSwapAction;
        using search2::BreadthFirstIterator;
        using search2::SearchNode;
        std::cout << "Filename: " << filename << std::endl;
        // we simply need to apply bfs, but only add a cost when a tile
        // from the group is moved, otherwise the moves do not cost anything
        uint8_t tiles[SIZE];
        for(auto i = 0; i < SIZE; ++i)
            tiles[i] = i;
        SBPuzzle<H, W> p(tiles, in_groups[i]);
        // perform breadth first search
        BreadthFirstIterator<SBPuzzle<H, W>, EA> bfs_itr(p);
        while(!bfs_itr.done()) {
            auto node = bfs_itr.next(); // get the next node
            // find the table index of the node's state
            auto index = calculate_table_index(i, node.puzzle.get_tiles());
            std::cout << "Index: " << index << ", cost: " << node.path_cost << std::endl
                      << node.puzzle << std::endl;
            // insert the path cost only if it is less than one found so far
            // this is necessary because the hole position is not accounted for,
            // and multiple states & paths can lead to the same index
            if(node.path_cost < tables[i][index])
                tables[i][index] = node.path_cost; // insert the path cost so far
            // I believe BFS fails for some problem instances. Therefore,
            // for each problem I want to solve it using A* and compare the path cost
            SBPuzzle<H, W> p2 = node.puzzle;
            int cost = search2::a_star_search<SBPuzzle<H, W>, EA, MH<H, W>>(p2);
            for(auto a : p2.template possible_actions<EA>())
                std::cout << static_cast<int>(a.tpos) << "," << static_cast<int>(a.hpos) << " ";
            std::cout << std::endl;
            if(cost != node.path_cost) {
                std::cout << "Found cost: " << node.path_cost << ", A*: " << cost << std::endl;
            }
            std::cout << "----------------------" << std::endl;
        }
        // write the table to the file
        write_byte_array(tables[i], table_sizes[i], filename);
    }

    template <int H, int W>
    size_t DPDB<H, W>::calculate_table_index(int i, const uint8_t *tiles) const {
        // first, find the combination index
        auto s = boolview_begin(i, tiles);
        auto e = boolview_end(i, tiles);
        size_t comb_i = calculate_combindex(s, e, SIZE, group_counts[i]);
        // then the lexicographical index of the group elements
        uint8_t group_size = group_counts[i];
        uint8_t group_tiles[group_size];
        fill_group(i, tiles, group_tiles);
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
        size_t lex_i = calculate_lexindex(group_tiles, group_tiles + group_size);
        // calculate the total index and lookup the table
        return comb_i * factorial(group_size) + lex_i;
    }

    template <int H, int W>
    int DPDB<H, W>::lookup(const SBPuzzle<H, W> &p) const {
        return lookup(p.get_tiles());
    }

    template <int H, int W>
    int DPDB<H, W>::lookup(const uint8_t *tiles) const {
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
