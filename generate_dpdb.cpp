constexpr int H = 3, W = 3;
constexpr int SIZE = H*W;

#include <iostream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <cstring>
#include <cctype>

#include "dpdb.hpp"

uint8_t validate_groups(const uint8_t *groups) {
    const uint8_t *min, *max;
    min = std::min_element(groups, groups+SIZE);
    max = std::max_element(groups, groups+SIZE, [](auto x, auto y) { // ignore 255
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
    bool *values = new bool[val_size]();
    for(int i = 0; i < SIZE; ++i)
        values[groups[i]] = true;
    for(size_t i = 0; i < val_size; ++i)
        if(!values[i])
            throw std::invalid_argument("Groups should be from 0 to N-1, but a value is missing");
    return *max+1;
}

void parse_group_string(const char *group_str, uint8_t *out, size_t out_size) {
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
            if(i > out_size)
                throw std::invalid_argument("Buffer overflow, input group spec is too long");
        } else {
            throw std::invalid_argument("Unknown char in group spec");
        }
        ++group_str;
    }
    if(acc > 0) {
        out[i++] = static_cast<uint8_t>(acc);
        if(i > out_size)
            throw std::invalid_argument("Buffer overflow, input group spec is too long");
    }
    std::cout << i << std::endl;
    if(i < out_size)
        throw std::invalid_argument("Input group spec is too short");
}

int main(int argc, char *argv[]) {
    if(argc < 3) {
        std::cout << "usage (3x3): ./generate_dpdb 0,0,0,0,0,1,1,1,X f1 f2\n";
        return 0;
    }

    // I wanted to create the files given a directory, but my current version
    // of g++ does not yet have the standard <filesystem> header

    uint8_t groups[SIZE];
    parse_group_string(argv[1], groups, SIZE);
    for(auto x : groups)
        std::cout << ((int) x) << " ";
    std::cout << std::endl;
    uint8_t num_groups = validate_groups(groups);
    if(num_groups != argc-2)
        throw std::invalid_argument("Too few/too many files for groups");

    sbpuzzle::DPDB<H, W>::generate_and_save(groups, groups+SIZE, argv+2, argv+argc);

    return 0;
}
