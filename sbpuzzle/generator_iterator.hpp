#ifndef DENIZMSAYIN_SBLOCK_SBPUZZLE_GENERATOR_ITERATOR_HPP
#define DENIZMSAYIN_SBLOCK_SBPUZZLE_GENERATOR_ITERATOR_HPP

#include "defs.hpp"

// TODO: this class does not act properly, since g is modified
// i.e. postfix ++ would not work because g is modified. Think
// of a way to handle this.

namespace denizmsayin::sblock::sbpuzzle::details {
    // a templated iterator class that goes with my custom generator template
    template <typename generator_type, typename value_type>
    class GeneratorIterator {
    using Direction = details::Direction;
    public:
        using difference_type = ptrdiff_t;
        using pointer = const value_type *;
        using reference = const value_type &;
        using iterator_category = std::forward_iterator_tag;

        GeneratorIterator(generator_type *g) : gen(g) {}
        GeneratorIterator& operator++() { incr(); return *this; }
        // GeneratorIterator operator++(int) { auto ret = *this; incr(); return ret; }
        bool operator==(GeneratorIterator other) { return gen == other.gen; }
        bool operator!=(GeneratorIterator other) { return gen != other.gen; }
        value_type operator*() { return gen->value(); }
    private:
        generator_type *gen;

        void incr() {
            gen->advance();
            if(!gen->has_more())
                gen = nullptr;
        }
    };
}

#endif
