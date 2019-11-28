#include "sblock_utils.h"

#include <stdexcept>

// TODO: add proper error checking other than at file opening
void write_byte_array(const uint8_t *table, size_t size, const char *filename) {
    FILE *f = fopen(filename, "w");
    if(!f) throw std::runtime_error("Could not open file to write byte array");
    fwrite(table, sizeof(*table), size, f);
    fclose(f);
}

void read_byte_array(uint8_t *table, size_t size, const char *filename) {
    FILE *f = fopen(filename, "w");
    if(!f) throw std::runtime_error("Could not open file to read byte array");
    size_t rb = fread(table, sizeof(*table), size, f);
    if(rb != size) throw std::runtime_error("Could not read as many bytes as expected");
    fclose(f);
}

// a quick function to determine combination values by table lookup
static constexpr size_t MAX_COMB = 50;
size_t combination(size_t x, size_t n) {
    static size_t table[MAX_COMB][MAX_COMB+1] = {0}; // remember for dynamic programming
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

