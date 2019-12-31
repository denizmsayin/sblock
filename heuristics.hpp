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
        #ifdef W_TORCH
        DLMODEL
        #endif
    };

    // why not auto-generate from the maps? because they are unordered
    const std::vector<std::string> HEURISTIC_STRINGS {
        "mispl",
        "manhattan",
        "dpdb",
        "rpdb",
        "cpdb",
        #ifdef W_TORCH
        "dlmodel"
        #endif
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


    }

    const std::unordered_map<std::string, std::string> HEURISTIC_DESCR_MAP {
        {"mispl", "number of misplaced tiles in the puzzle"},
        {"manhattan", "manhattan distance to the solved state"},
        {"dpdb", "disjoint pattern database (from file)"},
        {"rpdb", "disjoint pattern database, reflected"},
        {"cpdb", "disjoint pattern database, combined with reflection"},
        #ifdef W_TORCH
        {"dlmodel", "TorchScript deep learning model"}
        #endif
    };

    HeuristicType str2heuristictype(const std::string &s) {
        static const std::unordered_map<std::string, HeuristicType> map {
            {"mispl", HeuristicType::MISPL},
            {"manhattan", HeuristicType::MANHATTAN},
            {"dpdb", HeuristicType::DPDB},
            {"rpdb", HeuristicType::RPDB},
            {"cpdb", HeuristicType::CPDB},
            #ifdef W_TORCH
            {"dlmodel", HeuristicType::DLMODEL}
            #endif
        };
        return map.at(s);
    }

}

// actual class implementations
namespace sbpuzzle {

    template <psize_t H, psize_t W>
    class Heuristic {
    public:
        typedef std::vector<SBPuzzle<H, W>> pvector_t;
        typedef std::vector<pcost_t> cvector_t;

        virtual ~Heuristic() {}

        virtual pcost_t operator()(const SBPuzzle<H, W> &h) = 0;

        template <typename RandomAccessIterator, typename OutputIterator>
        void op_par_base(RandomAccessIterator begin, 
                         RandomAccessIterator end,
                         OutputIterator o)
        {
            search2::BHFWrapper<SBPuzzle<H, W>, Heuristic&, pcost_t>(*this)(begin, end, o);
        }

        // had to stick with vector since we cannot have templated virtual functions
        virtual void operator()(typename pvector_t::const_iterator begin, 
                                typename pvector_t::const_iterator end,
                                cvector_t::iterator o) 
        {
            op_par_base(begin, end, o);
        }
        
        virtual void operator()(typename pvector_t::const_iterator begin, 
                                typename pvector_t::const_iterator end,
                                std::back_insert_iterator<cvector_t> o) 
        {
            op_par_base(begin, end, o);
        }

    };

    template <psize_t H, psize_t W>
    class MisplacedTileHeuristic : public Heuristic<H, W> {
    public:
        pcost_t operator()(const SBPuzzle<H, W> &p) {
            return p.num_misplaced_tiles();
        }
    };

    template <psize_t H, psize_t W>
    class ManhattanHeuristic : public Heuristic<H, W> {
    public:
        pcost_t operator()(const SBPuzzle<H, W> &p) {
            return p.manhattan_distance_to_solution();
        }
    };


    template <psize_t H, psize_t W>
    class PDBHeuristic : public Heuristic<H, W> {
    public:
        PDBHeuristic(PDB<H, W> *p) : db(p) {}

        pcost_t operator()(const SBPuzzle<H, W> &p) {
            return db->lookup(p);
        }
    private:
        std::unique_ptr<PDB<H, W>> db;
    };

    #ifdef W_TORCH
    template <psize_t H, psize_t W>
    class DLModelHeuristic : public Heuristic<H, W> {
    public:
        DLModelHeuristic(std::unique_ptr<DLModel<H, W>> &&p) : model(std::move(p)) {}

        pcost_t operator()(const SBPuzzle<H, W> &p) {
            return model->forward(p);
        }
        
        virtual void operator()(typename pvector_t::const_iterator begin, 
                                typename pvector_t::const_iterator end,
                                cvector_t::iterator o) 
        {
            model->template forwar<float, pcost_t>(begin, end, o);
        }
        
        virtual void operator()(typename pvector_t::const_iterator begin, 
                                typename pvector_t::const_iterator end,
                                std::back_insert_iterator<cvector_t> o) 
        {
            model->template forwar<float, pcost_t>(begin, end, o);
        }

    private:
        std::unique_ptr<DLModel<H, W>> model;
    };
    #endif

    template <psize_t H, psize_t W>
    std::unique_ptr<Heuristic<H, W>> heuristic_factory(HeuristicType type, 
                                                       const std::string &file_path="",
                                                       bool use_gpu=false) {
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
            #ifdef W_TORCH
            case T::DLMODEL:    
            { 
                torch::Device dev = use_gpu ? torch::kCUDA : torch::kCPU;
                p = new DLModelHeuristic(DLModel<H, W>::from_file(file_path, dev));
                break;
            }
            #endif
            default:            throw std::invalid_argument("Unknown heuristic type. If using dlmodel, did you compile with -DW_TORCH?");
        }
        return std::unique_ptr<Heuristic<H, W>>(p);
    }

}

#endif
