#ifndef DENIZMSAYIN_SBLOCK_SEARCH_FACTORY_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_FACTORY_HPP

#include "bfs.hpp"
#include "astar.hpp"
#include "wastar.hpp"
#include "bwastar.hpp"
#include "idastar.hpp"

namespace denizmsayin::sblock::search {

    enum class SearchType {
        BFS,
        IDDFS,
        ASTAR,
        ID_ASTAR,
        WEIGHTED_ASTAR,
        BATCH_WEIGHTED_ASTAR
    };

    SearchType str2searchtype(const std::string &s) {
        typedef SearchType T;
        static const std::unordered_map<std::string, SearchType> map {
            {"bfs", T::BFS},
            {"iddfs", T::IDDFS},
            {"a*", T::ASTAR},
            {"ida*", T::ID_ASTAR},
            {"wa*", T::WEIGHTED_ASTAR},
            {"bwa*", T::BATCH_WEIGHTED_ASTAR}
        };
        return map.at(s);
    }

    // why not auto-generate from the maps? because they are unordered
    const std::vector<std::string> SEARCH_STRINGS {
        "bfs",
        "iddfs",
        "a*",
        "ida*",
        "wa*",
        "bwa*"
    };

    template <class Puzzle, class HF>
    using SearchFunction = std::function<SearchResult(const Puzzle &, double, size_t, HF)>;

    template <class P, class A, class HF, class C = default_cost_t, 
              bool R = false, bool B = false>
    SearchFunction<P, HF> search_factory(SearchType type) {
        typedef SearchType T;
        switch(type) {
            case T::BFS:                    return [](const P &p, double w, size_t b, HF hf) {
                                                return breadth_first_search<P, A, C, R>(p);
                                            };
//            case T::IDDFS:                  return [](const P &p, double w, size_t b, HF hf) {
//                                                return iterative_deepening_dfs<P, A, C>(p);
//                                            };
            case T::ASTAR:                  return [](const P &p, double w, size_t b, HF hf) {
                                                return a_star_search<P, A, HF, C, R>(p, hf);
                                            };
            case T::ID_ASTAR:               return [](const P &p, double w, size_t b, HF hf) {
                                                return iterative_deepening_a_star<P, A, HF, 
                                                                                  C, R, B>(p, hf);
                                            };
            case T::WEIGHTED_ASTAR:         return [](const P &p, double w, size_t b, HF hf) {
                                                return weighted_a_star_search<P, A, HF, C, R>
                                                       (p, w, hf);
                                            };
            case T::BATCH_WEIGHTED_ASTAR:   return [](const P &p, double w, size_t b, HF hf) {
                                                return batch_weighted_a_star_search<P, A, HF, C>
                                                       (p, w, b, hf);
                                            };
            default:                        throw std::invalid_argument("Unknown search type.");
        }
    }

}
#endif
