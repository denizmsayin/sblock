#ifndef DENIZMSAYIN_SBLOCK_SEARCH_WASTAR_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_WASTAR_HPP
#include <queue>

#include "defs.hpp"
#include "basic_nodes.hpp"
#include "action_ext.hpp"

namespace denizmsayin::sblock::search {

    template <class Puzzle, class Action, class HeuristicFunc, 
              typename Cost = default_cost_t, bool Rev = false>
    SearchResult weighted_a_star_search(const Puzzle &start_puzzle, double w, 
                                        HeuristicFunc hf=HeuristicFunc()) 
    {
        // Since the standard library PQ does not have a decrease-key operation, we have
        // to perform a few tricks. In dijkstra's algorithm, we can simply reinsert
        // a node with a better cost, and simply discard visited nodes when popping
        // from the queue, as we are guaranteed to have found the shortest path to
        // a visited node. 
        // A* does not have such a strong guarantee. We are certain that we will find 
        // the shortest path to the GOAL, but not each and every state. Thus it is
        // possible to find a shorter path to a state that has already been visited.
        // To deal with this, we also need to keep track of the smallest cost we have
        // found for each state so far. If we find a path with a lower cost, we simply
        // have to act as if that state was not visited.
        Puzzle goal = start_puzzle.goal_state();

        if(start_puzzle == goal) return SearchResult(0, 0);

        // tracker for expanded nodes
        SeriesTrackedValue<size_t> exp_ctr(0, TRACKER_OPTS);

        // typedefs for the data structures 
        typedef std::unordered_map<Puzzle, Cost> hash_table_t;
        typedef std::pair<const Puzzle, Cost> *entry_t;
        typedef HNode<entry_t, Cost> node_t;
        typedef MaybeActionExtended<Rev, Action, node_t> mnode_t;
        typedef std::priority_queue<mnode_t, std::vector<mnode_t>, std::greater<mnode_t>> 
                pqueue_t;

        // set up the search
        hash_table_t visited;
        pqueue_t pq;
        bool dummy;
        typename hash_table_t::iterator itr;

        // insert the first state into the hash table with a positive cost
        // to ensure that it gets expanded
        std::tie(itr, dummy) = visited.emplace(start_puzzle, 0);
        maybe_emplace_action_extended_node<Rev, Action>(pq, std::nullopt, &(*itr), 0, 
                                                        hf(start_puzzle));
        while(!pq.empty()) {
            auto node = pq.top(); pq.pop();
            auto entry = node.entry;
            const Puzzle &p = entry->first;
            ++exp_ctr; // increase expanded nodes
            std::optional<Action> prev_reverse = maybe_reverse_action<Rev, Action>(node);
            for(const auto &action : p.template action_generator<Action>()) {
                if(action != prev_reverse) {
                    Puzzle new_p = p;
                    Cost step_cost = new_p.apply_action(action);
                    Cost new_path_cost = node.path_cost + step_cost;
                    if(new_p == goal)
                        return SearchResult(new_path_cost, exp_ctr.get_value());
                    auto lookup = visited.find(new_p);
                    bool not_exist = lookup == visited.end();
                    if(not_exist || new_path_cost < lookup->second) {
                        int new_est_cost = static_cast<Cost>(w * new_path_cost) + hf(new_p);
                        if(not_exist) { // new state that did not exist before
                            std::tie(itr, dummy) = visited.emplace(new_p, new_path_cost);
                            maybe_emplace_action_extended_node<Rev, Action>(
                                pq,
                                action, 
                                &(*itr), 
                                new_path_cost, 
                                new_est_cost);
                        } else { // new path cost is smaller than previous
                            lookup->second = new_path_cost;
                            maybe_emplace_action_extended_node<Rev, Action>(
                                pq,
                                action, 
                                &(*lookup), 
                                new_path_cost, 
                                new_est_cost);
                        }
                    }
                }
            }
        }
        details::throw_unreachable();
        return SearchResult(0, exp_ctr.get_value());
    }
}
#endif
