#ifndef __HEURISTICS_HPP__
#define __HEURISTICS_HPP__

#include <utility>
#include <vector>
#include <memory>

#include "search2.hpp"
#include "sbpuzzle.hpp"
#include "pdb.hpp"
#include "dpdb.hpp"
#include "reflectdpdb.hpp"
#include "crdpdb.hpp"

#ifdef W_TORCH
#include "dlmodel.hpp"
#endif

// how to add a new heuristic: 
//  - add the heuristic to the HeuristicType enum
//  - add to HEURISTIC_STRINGS
//  - add a conversion from it to HEURISTIC_TYPE_MAP
//  - add a description to HEURISTIC_DESCR_MAP
//  - add a case to the heuristic_factory function

// enumeration and string conversion for the heuristic factory
namespace sbpuzzle {

    enum class HeuristicType {
        MISPL,
        MANHATTAN,
        DPDB,
        RPDB,
        CPDB,
        DLMODEL
    };

    // why not auto-generate from the maps? because they are unordered
    const std::vector<std::string> HEURISTIC_STRINGS {
        "mispl",
        "manhattan",
        "dpdb",
        "rpdb",
        "cpdb",
        "dlmodel"
    };

    namespace details {
        template <typename T>
        std::vector<std::string> map2keys(const std::unordered_map<std::string, T> &m) {
            std::vector<std::string> keys;
            for(const auto &kv : m)
                keys.emplace_back(kv.first);
            std::sort(keys.begin(), keys.end());
            return keys;
        }

        const std::unordered_map<std::string, HeuristicType> HEURISTIC_TYPE_MAP {
            {"mispl", HeuristicType::MISPL},
            {"manhattan", HeuristicType::MANHATTAN},
            {"dpdb", HeuristicType::DPDB},
            {"rpdb", HeuristicType::RPDB},
            {"cpdb", HeuristicType::CPDB},
            {"dlmodel", HeuristicType::DLMODEL}
        };

    }

    const std::unordered_map<std::string, std::string> HEURISTIC_DESCR_MAP {
        {"mispl", "number of misplaced tiles in the puzzle"},
        {"manhattan", "manhattan distance to the solved state"},
        {"dpdb", "disjoint pattern database (from file)"},
        {"rpdb", "disjoint pattern database, reflected"},
        {"cpdb", "disjoint pattern database, combined with reflection"},
        {"dlmodel", "TorchScript deep learning model"}
    };

    HeuristicType str2heuristictype(const std::string &s) {
        return details::HEURISTIC_TYPE_MAP.at(s);
    }

}

// actual class implementations
namespace sbpuzzle {

    template <psize_t H, psize_t W>
    class Heuristic {
    public:
        virtual ~Heuristic() {}

        virtual uint8_t operator()(const SBPuzzle<H, W> &h) = 0;

        virtual void operator()(const std::vector<SBPuzzle<H, W>> &p, std::vector<int> &o) {
            search2::BHFWrapper<SBPuzzle<H, W>, Heuristic&>(*this)(p, o);
        }
    };

    template <psize_t H, psize_t W>
    class MisplacedTileHeuristic : public Heuristic<H, W> {
    public:
        uint8_t operator()(const SBPuzzle<H, W> &p) {
            return p.num_misplaced_tiles();
        }
    };

    template <psize_t H, psize_t W>
    class ManhattanHeuristic : public Heuristic<H, W> {
    public:
        uint8_t operator()(const SBPuzzle<H, W> &p) {
            return p.manhattan_distance_to_solution();
        }
    };


    template <psize_t H, psize_t W>
    class PDBHeuristic : public Heuristic<H, W> {
    public:
        PDBHeuristic(PDB<H, W> *p) : db(p) {}

        uint8_t operator()(const SBPuzzle<H, W> &p) {
            return db->lookup(p);
        }
    private:
        std::unique_ptr<PDB<H, W>> db;
    };

    /*
    template <psize_t H, psize_t W>
    class DLModelHeuristic : public Heuristic<H, W> {
    public:
        DLModelHeuristic(
    */

    template <psize_t H, psize_t W>
    std::unique_ptr<Heuristic<H, W>> heuristic_factory(HeuristicType type, 
                                                       const std::string &file_path="") {
        typedef HeuristicType T;
        Heuristic<H, W> *p = nullptr;
        switch(type) {
            case T::MISPL:      p = new MisplacedTileHeuristic<H, W>(); 
                                break;
            case T::MANHATTAN:  p = new ManhattanHeuristic<H, W>(); 
                                break;
            case T::DPDB:       p = new PDBHeuristic<H, W>(
                                        new DPDB<H, W>(DPDB<H, W>::from_file(file_path)));
                                break;
            case T::RPDB:       p = new PDBHeuristic<H, W>(
                                        new ReflectDPDBIndependent<H, W>(
                                            DPDB<H, W>::from_file(file_path)));
                                break;
            case T::CPDB:       p = new PDBHeuristic<H, W>(
                                        new CRDPDB<H, W>(
                                            DPDB<H, W>::from_file(file_path)));
                                break;
            default:            throw std::invalid_argument("Unknown heuristic type");
        }
        return std::unique_ptr<Heuristic<H, W>>(p);
    }

}

#endif
