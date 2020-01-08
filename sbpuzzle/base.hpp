#ifndef __SBPUZZLE_BASE_HPP__
#define __SBPUZZLE_BASE_HPP__

#include "defs.hpp"

namespace sbpuzzle {

    // I've elected to define the functions right in the declaration since
    // writing them separately takes time and I keep changing implementations...

    // Since some functions are exactly the same for cases where the hole is
    // in the group and not, we have a base class that prevents us from
    // repeating the same functions a few times. The class is only
    // instantiable by the derived classes as its constructor is protected.
    // The derived classes still have to implement constructors, possible_actions(),
    // and apply_action()
    template <psize_t H, psize_t W>
    class Base {
        public:
            
            const std::array<pcell_t, H*W> &get_tiles() const {
                return tiles;
            }

            bool operator==(const SBPuzzleBase &other) const {
                return details::tiles_equal<H, W>(tiles, other.tiles);
            }

            // Actions are returned over a generator like class
            // template <typename Action>
            // class generator {
            //     class iterator { ... };
            //     generator(...)
            //     iterator begin() const {...}
            //     iterator end() const {...}
            //  };
            //  generator<Action> action_generator() const;
            //  See the SBPuzzle implementation for an example.

            size_t hash() const {
                return details::tiles_hash<H, W>(tiles);
            }

            friend std::ostream &operator<<(std::ostream &s, const SBPuzzleBase<H, W> &p) {
                return details::tiles_stream<H, W>(s, p.tiles);
            }

            std::ostream &to_binary_stream(std::ostream &s) const {
                const char *out_ptr = reinterpret_cast<const char *>(&tiles[0]);
                size_t out_size = tiles.size() * sizeof(tiles[0]);
                return s.write(out_ptr, out_size);
            }

            static size_t tile_size_in_bytes() {
                return sizeof(tiles);
            }

            int manhattan_distance_to_solution() const {
                return details::tiles_manhattan_distance_to_solution<H, W>(tiles);
            }

            int num_misplaced_tiles() const {
                return details::tiles_count_misplaced<H, W>(tiles);
            }

            pcost_t apply_action(TileSwapAction a) {
                tiles[a.hpos] = tiles[a.tpos]; // move tile over the hole
                tiles[a.tpos] = HOLE; // tile is now the hole
                hole_pos = a.tpos;
                return 1; // cost is always unit
            }

            pcost_t apply_action(DirectionAction d) {
                int8_t off = details::OFFSETS<W>[static_cast<uint8_t>(d.dir)];
                uint8_t tile_pos = hole_pos + off;
                return apply_action(TileSwapAction(tile_pos, hole_pos));
            }

            // a small optimized function for applying a sequence of actions
            void apply_action_sequence(const std::vector<TileSwapAction> &as) {
                for(auto a : as)
                    tiles[a.hpos] = tiles[a.tpos];
                tiles[as.back().tpos] = HOLE;
                hole_pos = as.back().tpos;
            }

        protected:
            // only callable by derived classes
            SBPuzzleBase() : hole_pos(details::HOLE<H, W>) {
                details::tiles_correct_fill<H, W>(tiles);
            }

            static constexpr auto SIZE = details::SIZE<H, W>;
            static constexpr auto HOLE = details::HOLE<H, W>;

            array<pcell_t, SIZE> tiles;
            psize_t hole_pos;
    };

}

#endif
