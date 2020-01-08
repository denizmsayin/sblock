#include "sblock_utils.hpp"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>

// a quick function to determine combination values by table lookup
static constexpr size_t MAX_COMB = 50;
size_t combination(size_t x, size_t n) {
    static size_t table[MAX_COMB][MAX_COMB+1] = {{0}}; // remember for dynamic programming
    if(x == n || n == 0)
        return 1;
    else if(table[x][n] == 0)
        table[x][n] = combination(x-1, n-1) + combination(x-1, n);
    return table[x][n];
}

// another similar function for factorial values
static constexpr size_t MAX_FACTORIAL = 50;
size_t factorial(size_t n) {
    static size_t table[MAX_FACTORIAL] = {0};
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


