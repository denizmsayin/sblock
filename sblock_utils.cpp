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

std::ostream& stream_tiles(std::ostream &s, const uint8_t *tiles, size_t h, size_t w, uint8_t x) {
    const uint8_t *max = std::max_element(tiles, tiles + h*w);
    uint8_t num_digits = (*max > 99) ? 3 : ((*max > 9) ? 2 : 1); 
    int num_dashes = w * (num_digits + 3) + 1;
    std::string dash_str(num_dashes, '-');
    std::string empty_str(num_digits - 1, ' ');
    for(size_t i = 0, k = 0; i < h; i++) {
        s << dash_str << std::endl;
        
        for(size_t j = 0; j < w; j++) {
            s << "| "; 
            if(tiles[k] == x) // marks masked tiles, big dep issue!
                s << empty_str << "X ";
            else 
                s << std::setw(num_digits) << static_cast<int>(tiles[k]) << " ";
            ++k;
        }
        s << "|" << std::endl;
    }
    s << dash_str;
    return s;
}

