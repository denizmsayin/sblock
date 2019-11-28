constexpr int H = 3, W = 3;
constexpr int SIZE = H*W;

#include <iostream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <stdexcept>
#include <algorithm>

#include "dpdb.hpp"

uint8_t validate_groups(const uint8_t *groups) {
    const uint8_t *min, *max;
    std::tie(min, max) = std::minmax_element(groups, groups + SIZE);
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
    std::stringstream ss(group_str);
    int64_t x;
    char ch;
    size_t i = 0;
    // input example for 3x3: 0,0,0,0,0,1,1,1,0
    while(ss >> x) { // extract a number 
        if(i == out_size)
            throw std::invalid_argument("Buffer overflow, input group spec is too long");
        ss >> ch; // discard ,
        if(x < 0 || x > 255)
            throw std::invalid_argument("Too large or negative values in input group spec");
        out[i++] = static_cast<uint8_t>(x);
    }
    if(i < out_size)
        throw std::invalid_argument("Input group spec is too short");
}

int main(int argc, char *argv[]) {
    if(argc < 3) {
        std::cout << "usage (3x3): ./generate_dpdb 0,0,0,0,0,1,1,1,0 f1 f2\n";
        return 0;
    }
    uint8_t groups[SIZE];
    parse_group_string(argv[1], groups, SIZE);
    uint8_t num_groups = validate_groups(groups);
    if(num_groups != argc-2)
        throw std::invalid_argument("Too few/too many files for groups");

    DPDB<H, W>::generate_and_save(groups, groups+SIZE, argv+2, argv+argc);

    return 0;
}
