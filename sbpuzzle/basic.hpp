#ifndef DENIZMSAYIN_SBLOCK_BASIC_HPP
#define DENIZMSAYIN_SBLOCK_BASIC_HPP

#include <array>

#include "defs.hpp"
#include "actions.hpp"
#include "tiles.hpp"
#include "generator_iterator.hpp"

namespace denizmsayin::sblock::sbpuzzle {

    // I've elected to define the functions right in the declaration since
    // writing them separately takes time and I keep changing implementations...

    // Since some functions are exactly the same for cases where the hole is
    // in the group and not, we have a base class that prevents us from
    // repeating the same functions a few times. The class is only
    // instantiable by the derived classes as its constructor is protected.
    // The derived classes still have to implement constructors, possible_actions(),
    // and apply_action()
    template <psize_t H, psize_t W>
    class Basic {
    protected:

        Basic() = default; // careful, uninitialized!

        static constexpr auto SIZE = details::SIZE<H, W>;
        static constexpr auto HOLE = details::HOLE<H, W>;
        static constexpr auto SERIALIZED_SIZE = SIZE * sizeof(pcell_t);

        // given_hole_pos is not necessarily equal to the stored hole pos
        // for non-basic cases, i.e. when the hole can propagate (see MaskedNoHole)
        void swap_hole(psize_t tile_pos, psize_t given_hole_pos) {
            tiles[given_hole_pos] = tiles[tile_pos]; // move the tile over the hole
            tiles[tile_pos] = HOLE; // move the hole over the tile
            hole_pos = tile_pos; // store the hole position
        }

        std::array<pcell_t, SIZE> tiles;
        psize_t hole_pos;

    public:

        Basic(const Basic &) = default;
        Basic(Basic &&) = default;

        // Usually, when searching, almost every single puzzle
        // will be constructed using the copy constructor.
        // Thus, applying checks in the default constructor seems
        // worthwhile. Or at least it can be made optional.
        // TODO: think about it
        template <typename InputIterator> 
        Basic(InputIterator begin) {
            std::copy_n(begin, SIZE, tiles.begin());
            hole_pos = details::tiles_find_hole<H, W>(tiles);
            details::tiles_validate<H, W>(tiles);
        }

        Basic(const std::array<pcell_t, SIZE> &o_tiles) 
            : Basic(o_tiles.begin()) {}

        // explicit factory for uninitialized, to prevent accidents
        static Basic uninitialized() {
            return Basic();
        }

        const std::array<pcell_t, H*W> &get_tiles() const {
            return tiles;
        }

        bool operator==(const Basic &other) const {
            return details::tiles_equal<H, W>(tiles, other.tiles);
        }

        size_t hash() const {
            return details::tiles_hash<H, W>(tiles);
        }

        // TODO: Are these really necessary as members?
        // Think about decorators.
        int manhattan_distance_to_solution() const {
            return details::tiles_manhattan_distance_to_solution<H, W>(tiles);
        }

        int num_misplaced_tiles() const {
            return details::tiles_count_misplaced<H, W>(tiles);
        }

        pcost_t apply_action(TileSwapAction a) {
            swap_hole(a.tpos, a.hpos);
            return 1; // cost is always unit
        }

        pcost_t apply_action(DirectionAction d) {
            int8_t off = details::OFFSETS<W>[static_cast<uint8_t>(d.dir)];
            uint8_t tile_pos = hole_pos + off;
            swap_hole(tile_pos, hole_pos);
            return 1;
        }

        // a small optimized function for applying a sequence of actions
        void apply_action_sequence(const std::vector<TileSwapAction> &as) {
            for(auto a : as)
                tiles[a.hpos] = tiles[a.tpos];
            tiles[as.back().tpos] = HOLE;
            hole_pos = as.back().tpos;
        }

        friend std::ostream &operator<<(std::ostream &s, const Basic<H, W> &p) {
            return details::tiles_stream<H, W>(s, p.tiles);
        }

        std::ostream &to_binary_stream(std::ostream &s) const {
            const char *out_ptr = reinterpret_cast<const char *>(&tiles[0]);
            return s.write(out_ptr, SERIALIZED_SIZE);
        }

        static Basic from_binary_stream(std::istream &stream) {
            // TODO: could use istream iterators? Also update for different pcell_t
            std::array<pcell_t, H*W> tiles;
            stream.read(reinterpret_cast<char *>(tiles.data()), SERIALIZED_SIZE);
            return MaskedWithHole(tiles);
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

        // TODO: think about public/private for the nested classes
        // Works for both TileSwapAction & DirectionAction, thanks to the tagged ctor
        template <class Action>
        class Generator {
        public:
        
            Generator(psize_t hp) : rec(&details::DIRECTION_RECORDS<H, W>[hp]), 
                index(hp), i(0) {}

            Generator(const Generator &) = delete;
            Generator(Generator &&) = default;

            Generator &operator=(const Generator &) = delete;
            Generator &operator=(Generator &&) = default;

            // standard Generator trio
            bool has_more() const { return i < rec->size; }
            void advance() { ++i; }
            Action value() const {
                constexpr static details::WTag<W> TAG; 
                return Action(TAG, index, rec->dirs[i]);
            }

            details::GeneratorIterator<Generator, Action> begin() { 
                return details::GeneratorIterator<Generator, Action>(this); 
            }

            details::GeneratorIterator<Generator, Action> end() const { 
                return details::GeneratorIterator<Generator, Action>(nullptr); 
            }

        private:
            // the only necessary state is the hole position
            const details::Record *rec;
            psize_t index;
            uint8_t i;
        };

        template <class Action>
        Generator<Action> action_generator() const {
            return Generator<Action>(hole_pos);
        }

    };

}

// Add hash to std, TODO: might be better to provide a custom hasher in the search.hpp templates
namespace std {
    template <denizmsayin::sblock::sbpuzzle::psize_t H, denizmsayin::sblock::sbpuzzle::psize_t W>
    struct hash<denizmsayin::sblock::sbpuzzle::Basic<H, W>> 
        : denizmsayin::sblock::sbpuzzle::details::hash<denizmsayin::sblock::sbpuzzle::Basic<H, W>> {};
};

#endif
