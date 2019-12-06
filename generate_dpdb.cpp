// H & W are defined so that they can be set from the Makefile
#ifndef __H
#define __H 3
#endif
#ifndef __W
#define __W 3
#endif

#include <iostream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <cstring>
#include <cctype>

#include "dpdb.hpp"

constexpr sbpuzzle::psize_t H = __H, W = __W;
constexpr int SIZE = H*W;

uint8_t validate_groups(const std::array<uint8_t, SIZE> &groups) {
    const uint8_t *min, *max;
    min = std::min_element(groups.begin(), groups.end());
    max = std::max_element(groups.begin(), groups.end(), [](auto x, auto y) { // ignore 255
            x = (x == sbpuzzle::DONT_CARE) ? 0 : x;
            y = (y == sbpuzzle::DONT_CARE) ? 0 : y;
            return x < y;
    });
    if(*min != 0)
        throw std::invalid_argument("Smallest group number is not 0");
    if(*max > SIZE-1)
        throw std::invalid_argument("Largest group number implies more groups than tiles");
    // now, ensure all groups from 0 to *max exist
    size_t val_size = *max + 1;
    std::vector<uint8_t> values(val_size, 0);
    for(int i = 0; i < SIZE; ++i)
        if(groups[i] != sbpuzzle::DONT_CARE)
            values[groups[i]] = 1;
    for(size_t i = 0; i < val_size; ++i)
        if(!values[i])
            throw std::invalid_argument("Groups should be from 0 to N-1, but a value is missing");
    return *max+1;
}

void parse_group_string(const char *group_str, std::array<uint8_t, SIZE> &out) {
    size_t i = 0;
    uint64_t acc = 0;
    // input example for 3x3: 0,0,0,0,0,1,1,1,X
    while(*group_str) {
        if(isdigit(*group_str)) {
            acc = 10*acc + (*group_str - '0');
            if(acc > 255)
                throw std::invalid_argument("Too large (>255) value in input group spec");
        } else if(*group_str == 'X') {
            acc = sbpuzzle::DONT_CARE;
        } else if(*group_str == ',') {
            out[i++] = static_cast<uint8_t>(acc);
            acc = 0;
            if(i > SIZE)
                throw std::invalid_argument("Buffer overflow, input group spec is too long");
        } else {
            throw std::invalid_argument("Unknown char in group spec");
        }
        ++group_str;
    }
    if(acc > 0) {
        out[i++] = static_cast<uint8_t>(acc);
        if(i > SIZE)
            throw std::invalid_argument("Buffer overflow, input group spec is too long");
    }
    if(i < SIZE)
        throw std::invalid_argument("Input group spec is too short");
}

int main(int argc, char *argv[]) {
    if(argc != 3) {
        std::cout << "usage (3x3): ./generate_dpdb 0,0,0,0,0,1,1,1,X filename\n";
        return 0;
    }

    // I wanted to create the files given a directory, but my current version
    // of g++ does not yet have the standard <filesystem> header

    std::array<uint8_t, SIZE> groups;
    parse_group_string(argv[1], groups);
    validate_groups(groups);

    sbpuzzle::DPDB<H, W>::generate_and_save(groups, argv[2]);

    return 0;
}
