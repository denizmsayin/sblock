#ifndef __HEURISTICS_HPP__
#define __HEURISTICS_HPP__

#include <utility>
#include <vector>
#include <memory>

#include "search2.hpp"
#include "sbpuzzle.hpp"


// enumeration and string conversion for the heuristic factory
namespace sbpuzzle {

    enum class HeuristicType {
        MISPL,
        MANHATTAN,
        PDB,
        RDB,
        CDB,
        DLMODEL
    };

    HeuristicType str2heuristictype(const std::string &s) {
        typedef HeuristicType T;
        static std::unordered_map<std::string, HeuristicType> map {
            {"mispl", T::MISPL},
            {"manhattan", T::MANHATTAN},
            {"pdb", T::PDB},
            {"rdb", T::RDB},
            {"cdb", T::CDB},
            {"dlmodel", T::DLMODEL}
        };
        return map[s];
    }

}

// actual class implementations
namespace sbpuzzle {

    template <psize_t H, psize_t W>
    class Heuristic {
    public:
        virtual uint8_t operator()(const SBPuzzle<H, W> &h) = 0;

        virtual void operator()(const std::vector<SBPuzzle<H, W>> &p, std::vector<int> &o) {
            search2::BHFWrapper<SBPuzzle<H, W>, Heuristic&>(*this)(p, o);
        }
    };

    template <psize_t H, psize_t W>
    class ManhattanHeuristic : public Heuristic<H, W> {
    public:
        uint8_t operator()(const SBPuzzle<H, W> &p) {
            return p.manhattan_distance_to_solution();
        }
    };

    template <psize_t H, psize_t W, typename... Args>
    std::unique_ptr<Heuristic<H, W>> heuristic_factory(HeuristicType type, Args&&... args) {
        typedef HeuristicType T;
        Heuristic<H, W> *p = nullptr;
        switch(type) {
            case T::MANHATTAN:  p = new ManhattanHeuristic<H, W>(std::forward<Args>(args)...); 
                                break;
            default:            throw std::invalid_argument("Unknown heuristic type");
        }
        return std::unique_ptr<Heuristic<H, W>>(p);
    }

}

#endif
