#include "sblock_utils.hpp"
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
    fread(table, sizeof(*table), size, f);
    fclose(f);
}
