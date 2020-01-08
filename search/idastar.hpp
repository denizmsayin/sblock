#ifndef DENIZMSAYIN_SBLOCK_SEARCH_IDASTAR_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_IDASTAR_HPP
#include "basic_nodes.hpp"
#include "action_ext.hpp"

namespace denizmsayin::sblock::search {
    
    namespace details {

        template <typename Puzzle, typename HeuristicFunc, typename Cost>
        struct CLDFSRec {
            const Puzzle goal;
            Cost cost_limit;
            SeriesTrackedValue<size_t> exp_ctr;
            HeuristicFunc heuristic;

            // Only used for creating the initial state of the record
            CLDFSRec(const Puzzle &p, HeuristicFunc hf) 
                : goal(p.goal_state()), cost_limit(hf(p)), 
                  exp_ctr(0, TRACKER_OPTS), heuristic(hf) {}
        };

        // TODO: this implementation of IDA* cannot decide on NOT_FOUND
        template <class Puzzle, class Action, class HeuristicFunc, class Cost, 
                  bool Rev, class Node, bool Backtrack = true>
        Cost cost_limited_dfs(typename std::conditional<Rev,
                                                        ActionExtended<Action, Node>,
                                                        Node>::type &node, 
                              CLDFSRec<Puzzle, HeuristicFunc, Cost> &record)
        {
            static_assert(!Backtrack || (Backtrack && Rev), 
                          "Backtracking cannot be enabled without reversing");
            if(node.est_cost > record.cost_limit) { // return the cutoff cost
                return node.est_cost;
            } else if(node.entry == record.goal) { // return the solution
                return node.path_cost;;
            } else {
                ++record.exp_ctr; // increment the expansion counter
                Cost min_exceeding_cost = std::numeric_limits<Cost>::max(); 
                std::optional<Action> prev_reverse = 
                    maybe_reverse_action<Rev, Action>(node);
                // saved variables for backtracking
                Cost last_path_cost = node.path_cost;
                Cost last_est_cost = node.est_cost;
                std::optional<Action> last_action = 
                    maybe_retrieve_action<Rev, Action>(node);
                for(const auto &action : node.entry.template action_generator<Action>()) {
                    // if no previous action was input, continue
                    // otherwise, do not perform the reverse of the previous action
                    // as it leads back to the same node and slows down
                    if(action != prev_reverse) {
                        Cost result;
                        if constexpr (Backtrack) {
                            // apply everything in-place
                            node.path_cost += node.entry.apply_action(action);
                            node.est_cost = node.path_cost + record.heuristic(node.entry);
                            node.action = action;
                            result = cost_limited_dfs<Puzzle, Action, HeuristicFunc, 
                                                      Cost, Rev, Node, Backtrack>(node, record);
                            node.entry.apply_action(Action::reverse(action));
                            node.path_cost = last_path_cost;
                            node.est_cost = last_est_cost;
                            node.action = last_action;
                        } else {
                            // generate parameters for a recursive call
                            Puzzle new_p = node.entry; 
                            Cost new_path_cost = node.path_cost + new_p.apply_action(action);
                            Cost new_est_cost = new_path_cost + record.heuristic(new_p);
                            auto new_node = 
                                maybe_construct_action_extended_node<Rev, Action, Node>(
                                    action, 
                                    new_p, 
                                    new_path_cost, 
                                    new_est_cost);
                            // call cost limited dfs on the neighbor
                            result = cost_limited_dfs<Puzzle, Action, HeuristicFunc, 
                                                      Cost, Rev, Node, Backtrack>(new_node, record); 
                        }
                        if(result <= record.cost_limit) // found
                            return result;
                        else if(min_exceeding_cost > result) // not found, but less than smallest exceeding
                            min_exceeding_cost = result;
                    }
                }
                return min_exceeding_cost;
            }
        }
    }

    template <class Puzzle, class Action, class HeuristicFunc, 
              class Cost = default_cost_t, bool Rev = false, bool Backtrack = false>
    SearchResult iterative_deepening_a_star(const Puzzle &p, HeuristicFunc hf=HeuristicFunc()) {
        using details::cost_limited_dfs;
        typedef HNode<Puzzle, Cost> Node;
        auto start_node = maybe_construct_action_extended_node<Rev, Action, Node>(
                std::nullopt, p, 0, hf(p));
        details::CLDFSRec<Puzzle, HeuristicFunc, Cost> cldfs_record(p, hf);
        while(true) {
            Cost result = cost_limited_dfs<Puzzle, Action, HeuristicFunc, Cost, 
                                           Rev, Node, Backtrack>(start_node, cldfs_record); 
            if(result <= cldfs_record.cost_limit)
                return SearchResult(result, cldfs_record.exp_ctr.get_value());
            cldfs_record.cost_limit = result;
        }
        return SearchResult(0, cldfs_record.exp_ctr.get_value());
    }
}

#endif
