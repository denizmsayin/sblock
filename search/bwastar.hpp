#ifndef DENIZMSAYIN_SBLOCK_SEARCH_BWASTAR_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_BWASTAR_HPP

#include <unordered_map>
#include <queue>
#include <algorithm>
#include <functional>

#include "defs.hpp"
#include "basic_nodes.hpp"

// TODO: add optional reverse with action_ext

namespace denizmsayin::sblock::search {
    // I am very confused about what to pass as parameters. I'd be up for passing
    // all HeuristicFunctions by const-reference, but there are cases where my heuristic
    // functions are stateful and can't be const. For example the DLModel case, where
    // the jit::script::Module does not remain const during inference. However, I have
    // all the freedom I need since HeuristicFunc is a template parameter, it can be
    // passed by non-const ref if I need it by specificying it as such

    template <class Puzzle, class Action, class BatchHeuristicFunc, 
              class Cost = default_cost_t, bool Rev = false>
    SearchResult batch_weighted_a_star_search(const Puzzle &start_puzzle, 
                                              double weight, 
                                              size_t batch_size, 
                                              BatchHeuristicFunc bhf=BatchHeuristicFunc()) 
    {
        // BatchHeuristicFunc: bhf(const std::vector<Puzzle> &in, std::vector<int> &out)
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

        // set up an expanded node counter and its tracker
        SeriesTrackedValue<size_t> exp_ctr(0, TRACKER_OPTS);

        // the standard states that pointers/refs to both keys and values remain
        // unaffected. Since pairs are stored (defined as value_type), using a 
        // pointer to pair should be fine, I believe.

        // typedefs for the data structures 
        typedef std::unordered_map<Puzzle, Cost> hash_table_t;
        typedef std::pair<const Puzzle, Cost> *entry_t;
        typedef HNode<entry_t, Cost> node_t;
        typedef std::priority_queue<node_t, std::vector<node_t>, std::greater<node_t>> pqueue_t;
        typedef decltype(start_puzzle.template action_generator<Action>()) gen_t;
        typedef decltype(start_puzzle.template action_generator<Action>().begin()) gen_t_itr;

        // set up the search
        hash_table_t visited;
        pqueue_t pq;
        bool dummy;
        typename hash_table_t::iterator itr;

        std::vector<Puzzle> batch_puzzles(batch_size);
        std::vector<Cost> batch_h_costs(batch_size), batch_p_costs(batch_size);
        std::vector<entry_t> batch_entries(batch_size);

        // insert the first state as the first batch
        batch_puzzles[0] = start_puzzle;
        bhf(batch_puzzles.begin(), batch_puzzles.begin() + 1, batch_h_costs.begin());
        std::tie(itr, dummy) = visited.emplace(start_puzzle, 0);
        pq.emplace(&(*itr), 0, batch_h_costs[0]);

        // since we have to expand a constant amount of nodes,
        // pq.empty() is not the only condition, we may have a
        // puzzle and action generator left from the previous iteration
        bool agen_exists = true;
        gen_t agen = start_puzzle.template action_generator<Action>();
        gen_t_itr agen_itr = agen.begin(), 
                  agen_end = agen.end();
        node_t node = pq.top(); pq.pop();
        ++exp_ctr;
        while(agen_exists || !pq.empty()) {
            size_t bi = 0;
            // might not be able to expand as many as batch_size nodes depending on cur. state
            while(bi < batch_size && (agen_exists || !pq.empty())) {
                // create a new generator from the top puzzle if there is none
                if(!agen_exists) {
                    node = pq.top(); pq.pop();
                    ++exp_ctr;
                    agen_exists = true;
                    agen = node.entry->first.template action_generator<Action>();
                    agen_itr = agen.begin();
                    agen_end = agen.end();
                }

                // generate a new puzzle from the from the current action
                // and add it to the eval list
                auto action = *agen_itr;
                ++agen_itr;
                Puzzle new_p = node.entry->first;
                Cost step_cost = new_p.apply_action(action);
                Cost new_path_cost = node.path_cost + step_cost;
                if(new_p == goal)
                    return SearchResult(new_path_cost, exp_ctr.get_value());
                auto lookup = visited.find(new_p);
                if(lookup == visited.end()) // store pointers if state already exists
                    batch_entries[bi] = nullptr;
                else
                    batch_entries[bi] = &(*lookup);
                batch_puzzles[bi] = new_p;
                batch_p_costs[bi] = new_path_cost;

                // increment the counter and mark generator end if necessary
                ++bi;
                if(agen_itr == agen_end)
                    agen_exists = false;
            }

            // evaluate the heuristic on bi puzzles, as much as the current batch
            bhf(batch_puzzles.begin(), batch_puzzles.begin() + bi, batch_h_costs.begin());

            // now, add the new states to the pq
            for(size_t i = 0; i < bi; ++i) {
                Cost new_est_cost = static_cast<Cost>(weight * batch_p_costs[i]) 
                                    + batch_h_costs[i];
                if(batch_entries[i] == nullptr) { // new state, must add
                    std::tie(itr, dummy) = visited.emplace(batch_puzzles[i], new_est_cost);
                    pq.emplace(&(*itr), batch_p_costs[i], new_est_cost);
                } else if(batch_p_costs[i] < batch_entries[i]->second) { 
                    // old state, check if value is exceeded
                    batch_entries[i]->second = batch_p_costs[i];
                    pq.emplace(&(*batch_entries[i]), batch_p_costs[i], new_est_cost);
                }
            }

        }

        details::throw_unreachable();
        return SearchResult(0, exp_ctr.get_value());
    }

    // wrapper class transforming single valued heuristic function to batch
    template <class Puzzle, class HF, class Cost = int64_t>
    class BHFWrapper {
    public:
        BHFWrapper(HF _hf=HF()) : hf(_hf) {}

        template <typename RandomAccessIterator, typename OutputIterator>
        void operator()(RandomAccessIterator begin,
                        RandomAccessIterator end,
                        OutputIterator out) 
        {
            std::transform(begin, end, out, std::ref(hf));
        }

    private:
        HF hf;
    };
}

#endif
