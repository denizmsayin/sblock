#ifndef DENIZMSAYIN_SBLOCK_SEARCH_BFS_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_BFS_HPP
#include <queue>

#include "defs.hpp"
#include "basic_nodes.hpp"
#include "action_ext.hpp"

namespace denizmsayin::sblock::search {
    // Puzzle and Action types are self explanatory, simple template arguments for the
    // functions. Cost is the type to use to store costs, which can be important
    // to provide memory savings. e.g. uint8_t can be used for low cost problems.
    // Rev is a template argument provided to take advantage of actions that contain
    // static Action reverse(Action a);
    // and prevents the checking of the reverse of the previous action when enabled,
    // which improves runtime.
    template <class Puzzle, class Action, typename Cost = default_cost_t, bool Rev = false>
    SearchResult breadth_first_search(const Puzzle &puzzle_start) {
        Puzzle goal = puzzle_start.goal_state();

        // early exit in case the puzzle is already solved
        if(puzzle_start == goal) return SearchResult(0, 0);

        // tracker for the expanded nodes
        SeriesTrackedValue<size_t> exp_ctr(0, TRACKER_OPTS);

        // typedefs for the data structures 
        // the plan is to keep all puzzle objects in the hash table,
        // and only store pointers in the queue. This is possible
        // because the standard dictates that while iterators
        // can be invalidated by insertion, pointers/references
        // to hash table elements remain valid. Thanks, separate chaining!
        typedef std::unordered_set<Puzzle> hash_table_t;
        typedef PNode<const Puzzle *, Cost> node_t;
        typedef typename MaybeActionExtended<Rev, Action, Node> mnode_t;
        typedef std::queue<mnode_t> queue_t;

        // data structures
        hash_table_t visited;
        queue_t q;
        bool dummy;
        typename hash_table_t::iterator itr;

        // no previous action, zero cost
        std::tie(itr, dummy) = visited.emplace(puzzle_start);
        maybe_emplace_action_extended_node<Rev, Action>(q, std::nullopt, &(*itr), 0); 
        while(!q.empty()) {
            ++exp_ctr;
            auto node = q.front(); q.pop();
            const Puzzle *p = node.entry;
            std::optional<Action> prev_reverse = maybe_reverse_action<Rev, Action>(node);
            for(const auto &action : p->template action_generator<Action>()) 
            {
                if(action != prev_reverse) {
                    Puzzle new_p = *p;
                    Cost new_path_cost = node.path_cost + new_p.apply_action(action);
                    if(new_p == goal)
                        return SearchResult(new_path_cost, exp_ctr.get_value());
                    if(visited.find(new_p) == visited.end()) {
                        std::tie(itr, dummy) = visited.emplace(new_p);
                        // construct a node with the current action
                        maybe_emplace_action_extended_node<Rev, Action>(q, action, &(*itr), 
                                                                        new_path_cost);
                    }
                }
            }
        }
        details::throw_unreachable();
        return SearchResult(0, exp_ctr.get_value());
    }
}

#endif
