#ifndef DENIZMSAYIN_SBLOCK_MASKEDNOHOLE_HPP
#define DENIZMSAYIN_SBLOCK_MASKEDNOHOLE_HPP

namespace denizmsayin::sblock::sbpuzzle {
    // The derived class for the case where the hole is not masked
    template <psize_t H, psize_t W>
    class MaskedNoHole : public Basic<H, W> {
    private:
        using B = Basic<H, W>;
        
        // TODO: consider caching hole/tile positions for faster searching
        // or simply a better method for being usable with BFS
        void prop_hole() {
            details::tiles_prop_hole<H, W>(B::tiles, B::hole_pos);
        }

        void reprop_hole() { // re-do hole propagation after a hole position update
            details::tiles_reprop_hole<H, W>(B::tiles, B::hole_pos);
        }

    public:

        template <typename CellInputIterator, typename BoolRandomIterator>
        MaskedNoHole(CellInputIterator cbegin, BoolRandomIterator mask) : B(cbegin)
        {
            if(mask[B::HOLE])
                throw std::invalid_argument("Constructing MaskedNoHole, but hole is in"
                                            " the provided mask.");
            for(size_t i = 0; i < B::SIZE; ++i) 
                if(!mask[B::tiles[i]])
                    B::tiles[i] = DONT_CARE;
            prop_hole();
        }

        explicit MaskedNoHole(const std::array<pcell_t, H*W> &i_tiles, 
                              const std::array<bool, H*W> &mask)
            : MaskedNoHole(i_tiles.begin(), mask.begin()) {}

        MaskedNoHole(const MaskedNoHole &other) = default;
        MaskedNoHole &operator=(const MaskedNoHole &other) = default;

        MaskedNoHole goal_state() const {
            return details::goal_state<MaskedNoHole<H, W>, H, W>(B::tiles);
        }

        pcost_t apply_action(TileSwapAction a) {
            pcost_t cost = B::apply_action(a);
            reprop_hole(); // repropagate the hole
            return cost; // cost is always unit
        }

        template <typename Action, typename Dummy = void>
        class Generator;

        template <typename Action>
        Generator<Action> action_generator() const {
            return Generator<Action>(*this);
        }

    };


    template <psize_t H, psize_t W>
    template <typename Dummy>
    class MaskedNoHole<H, W>::Generator<TileSwapAction, Dummy> {
    public:
        using GeneratorIterator = details::GeneratorIterator<Generator, TileSwapAction>;

        Generator(const MaskedNoHole<H, W> &puzzle) 
            : tiles(puzzle.get_tiles().data()), index(0), found_pos(0), i(-1)
        {
            advance_until_possible_index();
            advance();
        }

        Generator(const Generator &) = delete;
        Generator(Generator &&) = default;

        Generator &operator=(const Generator &) = delete;
        Generator &operator=(Generator &&) = default;

        bool has_more() const {
            return index < details::SIZE<H, W>;
        }

        void advance() {
            using namespace details;
            // invariant: currently on cell where a move has been previously found at i
            ++i; // advance the index on the current cell
            while(index < SIZE<H, W>) {
                const Record &rec = DIRECTION_RECORDS<H, W>[index];
                // try finding a move on the current cell
                while(i < rec.size) {
                    // if there are still cells we can check around this one
                    Direction dir = rec.dirs[i];
                    psize_t check_pos = index + OFFSETS<W>[static_cast<uint8_t>(dir)];
                    if(tiles[check_pos] == HOLE<H, W>) { // can swap tiles if it is a hole
                        found_pos = check_pos;
                        return;
                    }
                    else {
                        ++i;
                    }
                }
                // current cell exhausted, get the next cell 
                ++index; // move one step further
                advance_until_possible_index();
                i = 0;
            }

        }

        TileSwapAction value() const {
            return TileSwapAction(index, found_pos);
        }

        GeneratorIterator begin() {
            return GeneratorIterator(this);
        }

        GeneratorIterator end() const {
            return GeneratorIterator(nullptr);
        }

    private:
        void advance_until_possible_index() { // move on to the next non-hole tile
            while(index < details::SIZE<H, W>  // keep moving until a viable tile
                  && tiles[index] == details::HOLE<H, W>) 
                ++index;
        }

        const pcell_t *tiles;
        psize_t index;
        psize_t found_pos;
        uint8_t i;

    };

}

namespace std {
    template <denizmsayin::sblock::sbpuzzle::psize_t H, denizmsayin::sblock::sbpuzzle::psize_t W>
    struct hash<denizmsayin::sblock::sbpuzzle::MaskedNoHole<H, W>> 
        : denizmsayin::sblock::sbpuzzle::details::hash<denizmsayin::sblock::sbpuzzle::MaskedNoHole<H, W>> {};
};

#endif
