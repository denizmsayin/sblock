#include <iostream>
#include <fstream>
#include <string>
#include <limits>
#include <vector>
#include <unordered_set>

#include "sblock_utils.hpp"

template <typename T, typename C>
void printc(const C &v) {
    for(const auto &x : v)
        std::cout << static_cast<T>(x) << " ";
    std::cout << std::endl;
}

size_t get_file_size(std::fstream &stream) {
    stream.ignore(std::numeric_limits<std::streamsize>::max());
    size_t size = stream.gcount();
    stream.clear();
    stream.seekg(std::fstream::beg);
    return size;
}

namespace std {
    template <>
    struct hash<std::vector<char>> {
        size_t operator()(const std::vector<char> &arr) const noexcept {
            return hash_byte_array(reinterpret_cast<const uint8_t *>(&arr[0]), arr.size());
        }
    };
}

int main(int argc, char *argv[]) {
    using namespace std;
    if(argc != 4) {
        cout << "Usage: ./cmp entry_size file1 file2" << endl;
        return 0;
    }
    size_t entry_size = std::stoull(argv[1]);
    fstream f1(argv[2], fstream::in | fstream::binary);
    fstream f2(argv[3], fstream::in | fstream::binary);
    size_t s1 = get_file_size(f1);
    size_t s2 = get_file_size(f2);
    if(s1 != s2) {
        cout << "File sizes not equal. " << argv[2] << ": " << s1
             << argv[3] << ": " << s2 << endl;
        return 0;
    }

    // TODO: probably could be done coolly with istream iterators
    size_t num_entries = s1 / entry_size;
    std::unordered_set<std::vector<char>> entries1;
    for(size_t i = 0; i < num_entries; ++i) {
        std::vector<char> entry1(entry_size);
        f1.read(&entry1[0], entry_size);
        entries1.emplace(entry1);
    }
    f1.close();

    for(size_t i = 0; i < num_entries; ++i) {
        std::vector<char> entry2(entry_size);
        f2.read(&entry2[0], entry_size);
        if(entries1.find(entry2) == entries1.end()) {
            cout << "Missing entry." << endl;
            printc<int>(entry2);
            return 0;
        }
        cout << '\r' << (i+1) << '/' << num_entries << flush;
    }
    cout << endl;
    f2.close(); 

    cout << "Equal." << endl;
    return 0;
}
