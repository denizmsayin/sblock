#ifndef DENIZMSAYIN_SBLOCK_SEARCH_DEFS_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_DEFS_HPP
#include <unordered_map>
#include <vector>

#include "../utils.hpp"

// TODO: add the option for path remembering, will require fast dynamic allocations

// While the goal of achieving maximal code reuse was accomplished in the previous version
// of search.hpp, some algorithms suffered due to search being a high performance task,
// and lots of unnecessary operations being done. In this second header I will try to do a retake
// where almost every search function is implemented separately in order to maximize the performance
// obtained from each of them
//
// A base class for both tree and graph search classes
// The class Puzzle has to implement the following functions:
//      Puzzle goal_state() const { 
//          returns an instance of the goal state 
//      }
//
//      static Puzzle uninitialized() {
//          returns an uninitialized instance of the puzzle, for quick allocs
//      }
//
//      bool operator==(const Puzzle &p) const { i
//          check equality of states, necessary for using hash tables & comparing with goal
//      }
//
//      template <typename Action>
//      generator_like action_generator() const {
//          return something that conforms to the C++ forward iterator interface,
//          its goal is to generate actions one after the other, so it must have
//          .begin() and .end() methods. Those methods should return an object
//          that act like a forward iterator and thus have the ++ & == operators.
//          In this code, the generators are mostly used with C++11's for each loop.
//          Can have different overloads for different action types.
//          i.e. for(const auto &action : puzzle.template action_generator<DirectionAction>()) {}
//      }
//
//      cost_t apply_action(Action a) { 
//          applies action a to the puzzle and returns the cost, cost_t can be 
//          any basic integral/floating type.
//      }
//
//      and must extend std::hash for using hash tables.
//
// The class Action must be used and returned by Puzzle as explained above. Puzzle and Action
// classes can have many to many relationships thanks to the templated action_generator()
// and static overloads on apply_action(Action a).
//
// For bidirectional search, as well as making use of direct reverse optimization (not applying
// moves leading back to the previous state), the following must be defined by the Action class:
//      static Action inverse(Action a); { 
//          returns the reverse of action a 
//      }
//
// The class HeuristicFunc will have the following signature:
//      cost_t HeuristicFunc(const Puzzle &p) { 
//          returns a heuristic value, lower is better 
//      }
//
// The BatchHeuristicFunc is a bit troublesome due to the lack of templated virtual functions.
// I have elected to define two possible input types for it:
// void BHF(std::vector<Puzzle>::const_iterator begin,
//          std::vector<Puzzle>::const_iterator end,
//          std::vector<cost_t>::iterator out)
// as well as
// void BHF(std::vector<Puzzle>::const_iterator begin,
//          std::vector<Puzzle>::const_iterator end,
//          std::back_insert_iterator<std::vector<cost_t>> out)
//

namespace denizmsayin::sblock::search {

    // Options for tracking generated node count, can be modified at runtime
    utils::SeriesTracker<size_t>::Options TRACKER_OPTS = utils::SeriesTracker<size_t>::Options{}
        .do_track(false)
        .print_every(1000000)
        .name_str("Nodes expanded");
   
    typedef int64_t default_cost_t;

    struct SearchResult {
        size_t cost;
        size_t nodes_expanded;

        SearchResult(size_t c, size_t ne) : cost(c), nodes_expanded(ne) {}
    };

    namespace details {
        // TODO: integrate into SearchResult rather than throwing an exception
        void throw_unreachable() {
            throw std::invalid_argument("The goal state is not reachable from the provided "
                                        "initial state");
        }
    }


}


#endif
