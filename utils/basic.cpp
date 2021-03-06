#include "basic.hpp"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>
#include <random>
#include <unordered_set>

namespace denizmsayin::sblock::utils {

    // a quick function to determine combination values by table lookup
    static constexpr size_t MAX_COMB = 50;
    unsigned long combination(unsigned long x, unsigned long n) {
        // remember for dynamic programming
        static unsigned long table[MAX_COMB][MAX_COMB+1] = {{0}}; 
        if(x == n || n == 0)
            return 1;
        else if(table[x][n] == 0)
            table[x][n] = combination(x-1, n-1) + combination(x-1, n);
        return table[x][n];
    }

    // another similar function for factorial values
    static constexpr size_t MAX_FACTORIAL = 50;
    unsigned long factorial(size_t n) {
        static unsigned long table[MAX_FACTORIAL] = {0};
        if(n == 0)
            return 1;
        else if(table[n] == 0)
            table[n] = n * factorial(n-1);
        return table[n];
    }


    // copied from the boost implementation
    static inline size_t hash_combine(size_t h1, size_t h2) {
        return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }

    template <typename P>
    static inline size_t cast_hash(const void *p) {
        return std::hash<P>()(*static_cast<const P *>(p));
    }

    static inline const void *offset_voidptr(const void *p, size_t x) {
        return static_cast<const void *>(reinterpret_cast<const char *>(p) + x);
    }

    size_t hash_byte_array(const uint8_t *arr, size_t size) {
        // first, do all you can using 8 bytes each time
        size_t seed = 0;
        constexpr std::hash<uint64_t> hasher64;
        size_t n = size;
        const uint64_t *itr = reinterpret_cast<const uint64_t *>(arr);
        while(n >= 8) {
            seed = hash_combine(seed, hasher64(*itr++));
            n -= 8;
        }
        // hash the remainder
        const void *p = reinterpret_cast<const void *>(itr);
        switch(n) {
            case 1: 
                seed = hash_combine(seed, cast_hash<uint8_t>(p)); 
                break;
            case 2: 
                seed = hash_combine(seed, cast_hash<uint16_t>(p)); 
                break;
            case 3: 
                seed = hash_combine(hash_combine(seed, cast_hash<uint16_t>(p)),
                        cast_hash<uint8_t>(offset_voidptr(p, 2)));
                break;
            case 4: 
                seed = hash_combine(seed, cast_hash<uint32_t>(p)); 
                break;
            case 5: 
                seed = hash_combine(hash_combine(seed, cast_hash<uint32_t>(p)),
                        cast_hash<uint8_t>(offset_voidptr(p, 4)));
                break;
            case 6:
                seed = hash_combine(hash_combine(seed, cast_hash<uint32_t>(p)),
                        cast_hash<uint16_t>(offset_voidptr(p, 4)));
                break;
            case 7:
                seed = hash_combine(hash_combine(hash_combine(seed, cast_hash<uint32_t>(p)),
                            cast_hash<uint16_t>(offset_voidptr(p, 4))),
                        cast_hash<uint8_t>(offset_voidptr(p, 6)));
                break;
            default: ;
        }
        return seed;
        // brought form 34 ms to 28 ms compared to hashing one by one
    }

    std::vector<size_t> generate_different_bitstrings(size_t n, unsigned seed) {
        std::mt19937_64 mt(seed); // mersenne twister RNG
        std::unordered_set<size_t> bitstrings;
        for(size_t i = 0; i < n; ++i) {
            bool not_done = true;
            while(not_done) {
                size_t bs = mt();
                if(bitstrings.find(bs) == bitstrings.end()) {
                    bitstrings.insert(bs);
                    not_done = false;
                }
            }
        }
        return std::vector<size_t>(bitstrings.begin(), bitstrings.end());
    }

}
