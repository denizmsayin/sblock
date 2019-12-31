#ifndef __SEARCH2_HPP__
#define __SEARCH2_HPP__

#include <algorithm>
#include <memory>
#include <vector>
#include <list>
#include <unordered_set>
#include <unordered_map>
#include <stdexcept>
#include <limits>
#include <queue>
#include <stack>
#include <iostream>
#include <chrono>
#include <functional>
#include <optional>

#include "sblock_utils.hpp"
#include "search_queues.hpp"

namespace search2 {
    
    typedef int64_t default_cost_t;

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
}


// TODO: update all comments
// add the option for path remembering, will require fast dynamic allocations
// along with the previous update, consider adding backtracking search as an alternative

// While the goal of achieving maximal code reuse was accomplished in the previous version
// of search.hpp, some algorithms suffered due to search being a high performance task,
// and lots of unnecessary operations being done. In this second header I will try to do a retake
// where almost every search function is implemented separately in order to maximize the performance
// obtained from each of them
//
// A base class for both tree and graph search classes
// The class Puzzle has to implement the following functions:
//     bool is_solved() const { returns true if the Puzzle is in a goal state, false o.w. }
//     Container possible_actions<Action>() const { 
//         returns possible actions for the current puzzle 
//         templated to allow multiple action sets for a puzzle
//     }
//     int apply_action(Action a) { applies 'a' to the puzzle and returns the cost of the action }
// the class must also define a hash function so that it can be stored in unordered_set.
// As an alternative for Container possible_actions() const, we will have a function returning
// an iterator and iterate over possible actions. This should make it slightly nicer, hopefully:
//     Puzzle::ActionIterator action_iterator() const { returns an iterator going over possible actions }
//     The iterator is expected to have the following interface:
//         bool done() const;
//         Action next_action();
//
// The class Action must be used and returned by Puzzle as explained above.
// For bidirectional search, the following function must also be defined:
//     Action inverse(Action a); { returns the reverse of action a }
//
// The class HeuristicFunc must be called as if it has the following signature:
//     int HeuristicFunc(const Puzzle &p) { returns a heuristic value, lower is better }
//
// The template<class, class> class Queue must support the following functions:
//     Queue() { empty constructor }
//     bool empty() const { returns true if the queue is empty, false o.w. }
//     void push(Record &r) { adds record r to the queue }
//     const Record &top() const { returns the top record }
//     void pop() { pops the top record }
//
// To increase code reuse, I decided to have both tree and graph search classes to inherit from
// the same class, and have visitation insertions/checks performed by virtual functions. Graph
// search does the insertions & checks in these functions, while the tree search simply skips
// them

// For the return value of the functions, I decided to return a vector of actions just as before
// While specifically using a vector is slightly restrictive, we can rely on return value
// optimization to avoid copying, and returning by value creates less of a burden
// for the caller who only has to assign the return value to a vector

// BREADTH FIRST SEARCH
// BFS is quite simple, and to reconstruct the path we simply need to keep a
// linked list (tree) of actions that go all the way back up to the first state

// BfsActionRecord keeps the previous action, while BfsNode is what's actually
// kept in the queue. The reason we have two different records is to save a
// bit of memory, once a state is removed from the queue we no longer need
// to remember the state itself, just the action that led to it so that
// we can reconstruct the solution path
namespace search2 {

    // Options for tracking generated node count, can be modified at runtime
    SeriesTracker<size_t>::Options TRACKER_OPTS = SeriesTracker<size_t>::Options{}
        .do_track(false)
        .print_every(1000000)
        .name_str("Nodes expanded");


    namespace details {
        void throw_unreachable() {
            throw std::invalid_argument("The goal state is not reachable from the provided initial state");
        }

    }

    // SEARCH NODE, the class that defines a search node, a record structure that keeps
    // both a puzzle state and extra book-keeping information about it.
    // It is a templated class because what is contained can change depending on the algorithm.
    // i.e. a const Puzzle * for BFS, or a pair<Puzzle, int> * for A* search etc.
    template <typename Contained, typename Cost = default_cost_t> 
    struct SearchNode {
        Contained entry;
        Cost path_cost;
        Cost est_cost;

        SearchNode(const Contained &p, Cost pc, Cost ec) 
            : entry(p), path_cost(pc), est_cost(ec) {}

        SearchNode(const Contained &p, Cost pc) 
            : entry(p), path_cost(pc), est_cost(0) {}
    };

    template <class Puzzle, class Cost>
    class SearchNodeComparator {
    public:
        bool operator()(const SearchNode<Puzzle, Cost> &n1, const SearchNode<Puzzle, Cost> &n2) {
            return n1.est_cost > n2.est_cost;
        }
    };

    template <class Puzzle, class Cost>
    class RevSearchNodeComparator {
    public:
        bool operator()(const SearchNode<Puzzle, Cost> &n1, const SearchNode<Puzzle, Cost> &n2) {
            return n1.est_cost < n2.est_cost;
        }
    };

    struct SearchResult {
        size_t cost;
        size_t nodes_expanded;

        SearchResult(size_t c, size_t ne) : cost(c), nodes_expanded(ne) {}
    };

    /*
    template <class Puzzle, class Action>
    class BreadthFirstIterator {
        public:
            BreadthFirstIterator(const Puzzle &p);
            bool done() const;
            SearchNode<Puzzle> next();

        private:
            std::unordered_set<Puzzle> visited;
            std::queue<SearchNode<Puzzle>> q;
    };

    template <class Puzzle, class Action>
    BreadthFirstIterator<Puzzle, Action>::BreadthFirstIterator(const Puzzle &p)
    : visited(), q()
    {
        q.emplace(p, 0);
        visited.emplace(p);
    }

    template <class Puzzle, class Action>
    bool BreadthFirstIterator<Puzzle, Action>::done() const {
        return q.empty();
    }

    template <class Puzzle, class Action>
    SearchNode<Puzzle> BreadthFirstIterator<Puzzle, Action>::next() {
        auto node = q.front(); q.pop();
        const Puzzle &p = node.entry;
        for(auto action : p.template possible_actions<Action>()) {
            Puzzle new_p = p;
            int new_path_cost = node.path_cost + new_p.apply_action(action);
            if(visited.find(new_p) == visited.end()) {
                visited.emplace(new_p);
                q.emplace(new_p, new_path_cost);
            }
        }
        return node;
    }
    */

    template <class Puzzle, class Action, typename Cost = default_cost_t>
    SearchResult breadth_first_search(const Puzzle &puzzle_start) {
        // early exit in case the puzzle is already solved
        if(puzzle_start.is_solved()) return SearchResult(0, 0);

        // tracker for the expanded nodes
        SeriesTrackedValue<size_t> exp_ctr(0, TRACKER_OPTS);

        // typedefs for the data structures 
        // the plan is to keep all puzzle objects in the hash table,
        // and only store pointers in the queue. This is possible
        // because the standard dictates that while iterators
        // can be invalidated by insertion, pointers/references
        // to hash table elements remain valid. Thanks, separate chaining!
        typedef std::unordered_set<Puzzle> hash_table_t;
        typedef SearchNode<const Puzzle *, Cost> node_t;
        typedef std::queue<node_t> queue_t;

        // data structures
        hash_table_t visited;
        queue_t q;
        bool dummy;
        typename hash_table_t::iterator itr;

        std::tie(itr, dummy) = visited.emplace(puzzle_start);
        q.emplace(&(*itr), 0);
        while(!q.empty()) {
            ++exp_ctr;
            node_t node = q.front(); q.pop();
            const Puzzle *p = node.entry;
            for(const auto &action : p->template action_generator<Action>()) 
            {
                Puzzle new_p = *p;
                Cost new_path_cost = node.path_cost + new_p.apply_action(action);
                if(new_p.is_solved())
                    return SearchResult(new_path_cost, exp_ctr.get_value());
                if(visited.find(new_p) == visited.end()) {
                    std::tie(itr, dummy) = visited.emplace(new_p);
                    q.emplace(&(*itr), new_path_cost);
                }
            }
        }
        details::throw_unreachable();
        return SearchResult(0, exp_ctr.get_value());
    }

    /*
    enum class IDFSResult {
        CUTOFF,
        FOUND,
        NOT_FOUND
    };

    template <class Puzzle, class Action>
    IDFSResult depth_limited_dfs(const Puzzle &p, int depth, int *exp_ctr) {
        if(p.is_solved()) {
            return IDFSResult::FOUND;
        } else if(depth == 0) {
            return IDFSResult::CUTOFF;
        } else {
            ++(*exp_ctr);
            bool cutoff = false;
            for(auto action : p.template possible_actions<Action>()) {
                Puzzle new_p = p; new_p.apply_action(action);
                IDFSResult result = depth_limited_dfs<Puzzle, Action>(new_p, depth-1, exp_ctr);
                if(result == IDFSResult::CUTOFF)
                    cutoff = true;
                else if(result != IDFSResult::NOT_FOUND) 
                    return IDFSResult::FOUND;
            }
            if(cutoff)
                return IDFSResult::CUTOFF;
            else
                return IDFSResult::NOT_FOUND;
        }
    }

    template <class Puzzle, class Action>
    SearchResult iterative_deepening_dfs(const Puzzle &p) {
        static int LIMIT = 1000000;
        int exp_ctr = 0;
        for(int depth = 0; depth < LIMIT; depth++) {
            IDFSResult result = depth_limited_dfs<Puzzle, Action>(p, depth, &exp_ctr);
            if(result == IDFSResult::FOUND)
                return SearchResult(depth, exp_ctr);
            else if(result == IDFSResult::NOT_FOUND) {
                details::throw_unreachable();
                return SearchResult(0, 0);
            }
        }
        throw std::invalid_argument("The goal state could not be reached in 1M steps");
    }
    */

    /*
    namespace details {
        template <class Puzzle, class Action>
            int step_single_direction(std::queue<SearchNode<Puzzle>> &queue,
                    std::unordered_map<Puzzle, int> &visited, 
                    std::unordered_map<Puzzle, int> &other_visited)
            {
                if(!queue.empty()) {
                    auto node = queue.front(); queue.pop();
                    const auto &p = node.entry;
                    // check if the node has been visited the other way
                    auto lookup = other_visited.find(p);
                    if(lookup != other_visited.end())
                        return node.path_cost + lookup->second;
                    // otherwise insert and generate new nodes like bfs
                    visited.emplace(p, node.path_cost);
                    for(auto action : p.template possible_actions<Action>()) {
                        Puzzle new_p = p; 
                        int path_cost = new_p.apply_action(action);
                        if(visited.find(new_p) == visited.end()) // not visited
                            queue.emplace(new_p, node.path_cost + path_cost);
                    }
                }
                return -1;
            }
    }

    template <class Puzzle, class Action>
        int bidirectional_bfs(const Puzzle &p) {
            using details::step_single_direction;
            std::unordered_map<Puzzle, int> f_visited, b_visited;
            std::queue<SearchNode<Puzzle>> fq, bq;
            fq.emplace(p, 0);
            bq.emplace(p.goal_state(), 0);
            while(!fq.empty() || !bq.empty()) {
                // forward step
                int r = step_single_direction<Puzzle, Action>(fq, f_visited, b_visited);
                if(r > 0) return r;
                // backward step
                r = step_single_direction<Puzzle, Action>(bq, b_visited, f_visited);
                if(r > 0) return r;
            }
            details::throw_unreachable(); 
            return 0;
        }
    */

    // I am very confused about what to pass as parameters. I'd be up for passing
    // all HeuristicFunctions by const-reference, but there are cases where my heuristic
    // functions are stateful and can't be const. For example the DLModel case, where
    // the jit::script::Module does not remain const during inference. However, I have
    // all the freedom I need since HeuristicFunc is a template parameter, it can be
    // passed by non-const ref if I need it by specificying it as such

    template <class Puzzle, class Action, class BatchHeuristicFunc, class Cost = default_cost_t>
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
       
        if(start_puzzle.is_solved()) return SearchResult(0, 0);

        // set up an expanded node counter and its tracker
        SeriesTrackedValue<size_t> exp_ctr(0, TRACKER_OPTS);

        // the standard states that pointers/refs to both keys and values remain
        // unaffected. Since pairs are stored (defined as value_type), using a 
        // pointer to pair should be fine, I believe.

        // typedefs for the data structures 
        typedef std::unordered_map<Puzzle, Cost> hash_table_t;
        typedef std::pair<const Puzzle, Cost> *entry_t;
        typedef SearchNode<entry_t, Cost> node_t;
        typedef SearchNodeComparator<entry_t, Cost> comp_t;
        typedef std::priority_queue<node_t, std::vector<node_t>, comp_t> pqueue_t;
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
                if(new_p.is_solved())
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


    template <class Puzzle, class Action, class HeuristicFunc, typename Cost = default_cost_t>
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
        if(start_puzzle.is_solved()) return SearchResult(0, 0);

        // tracker for expanded nodes
        SeriesTrackedValue<size_t> exp_ctr(0, TRACKER_OPTS);

        // typedefs for the data structures 
        typedef std::unordered_map<Puzzle, Cost> hash_table_t;
        typedef std::pair<const Puzzle, Cost> *entry_t;
        typedef SearchNode<entry_t, Cost> node_t;
        typedef SearchNodeComparator<entry_t, Cost> comp_t;
        typedef std::priority_queue<node_t, std::vector<node_t>, comp_t> pqueue_t;

        // set up the search
        hash_table_t visited;
        pqueue_t pq;
        bool dummy;
        typename hash_table_t::iterator itr;

        // insert the first state into the hash table with a positive cost
        // to ensure that it gets expanded
        std::tie(itr, dummy) = visited.emplace(start_puzzle, 0);
        pq.emplace(&(*itr), 0, hf(start_puzzle));

        while(!pq.empty()) {
            node_t node = pq.top(); pq.pop();
            entry_t entry = node.entry;
            const Puzzle &p = entry->first;
            ++exp_ctr; // increase expanded nodes
            for(const auto &action : p.template action_generator<Action>()) {
                Puzzle new_p = p;
                Cost step_cost = new_p.apply_action(action);
                Cost new_path_cost = node.path_cost + step_cost;
                if(new_p.is_solved())
                    return SearchResult(new_path_cost, exp_ctr.get_value());
                auto lookup = visited.find(new_p);
                bool not_exist = lookup == visited.end();
                if(not_exist || new_path_cost < lookup->second) {
                    int new_est_cost = static_cast<Cost>(w * new_path_cost) + hf(new_p);
                    if(not_exist) { // new state that did not exist before
                        std::tie(itr, dummy) = visited.emplace(new_p, new_path_cost);
                        pq.emplace(&(*itr), new_path_cost, new_est_cost);
                    } else { // new path cost is smaller than previous
                        lookup->second = new_path_cost;
                        pq.emplace(&(*lookup), new_path_cost, new_est_cost);
                    }
                }
            }
        }
        details::throw_unreachable();
        return SearchResult(0, exp_ctr.get_value());
    }

    template <class Puzzle, class Action, class HeuristicFunc, class Cost = default_cost_t>
    SearchResult a_star_search(const Puzzle &p, HeuristicFunc hf=HeuristicFunc()) {
        return weighted_a_star_search<Puzzle, Action, HeuristicFunc, Cost>(p, 1.0, hf);
    }

    namespace details {

        // TODO: this implementation of IDA* cannot decide on NOT_FOUND
        template <class Puzzle, class Action, class HeuristicFunc, class Cost>
        Cost cost_limited_dfs(const SearchNode<Puzzle, Cost> &node, 
                              Cost cost_limit, 
                              SeriesTrackedValue<size_t> &exp_ctr,
                              const Action *prev_action_reverse = nullptr,
                              HeuristicFunc hf=HeuristicFunc()) 
        {
            if(node.est_cost > cost_limit) { // return the cutoff cost
                return node.est_cost;
            } else if(node.entry.is_solved()) { // return the solution
                return node.path_cost;;
            } else {
                ++exp_ctr; // increment the expansion counter
                Cost min_exceeding_cost = std::numeric_limits<Cost>::max(); 
                for(const auto &action : node.entry.template action_generator<Action>()) {
                    // if no previous action was input, continue
                    // otherwise, do not perform the reverse of the previous action
                    // as it leads back to the same node and slows down
                    if(prev_action_reverse == nullptr || 
                       *prev_action_reverse != action) 
                    {
                        // generate parameters for a recursive call
                        Puzzle new_p = node.entry; 
                        Cost new_path_cost = node.path_cost + new_p.apply_action(action);
                        Cost new_est_cost = new_path_cost + hf(new_p);
                        SearchNode<Puzzle, Cost> new_node(new_p, new_path_cost, new_est_cost);
                        Action rev_action = Action::reverse(action);
                        // call cost limited dfs on the neighbor
                        Cost result = 
                            cost_limited_dfs<Puzzle, Action, HeuristicFunc>(new_node, 
                                                                            cost_limit, 
                                                                            exp_ctr, 
                                                                            &rev_action, 
                                                                            hf);
                        if(result <= cost_limit) // found
                            return result;
                        else if(min_exceeding_cost > result) // not found, but less than smallest exceeding
                            min_exceeding_cost = result;
                    }
                }
                return min_exceeding_cost;
            }
        }
    }

    template <class Puzzle, class Action, class HeuristicFunc, class Cost = default_cost_t>
    SearchResult iterative_deepening_a_star(const Puzzle &p, HeuristicFunc hf=HeuristicFunc()) {
        using details::cost_limited_dfs;
        Cost cost_limit = hf(p);
        SearchNode<Puzzle, Cost> start_node(p, 0, cost_limit);
        SeriesTrackedValue<size_t> exp_ctr(0, TRACKER_OPTS);
        while(true) {
            Cost result = cost_limited_dfs<Puzzle, Action, HeuristicFunc, Cost>(start_node, 
                                                                                cost_limit, 
                                                                                exp_ctr, 
                                                                                nullptr,
                                                                                hf);
            if(result <= cost_limit) 
                return SearchResult(result, exp_ctr.get_value());
            cost_limit = result;
        }
        return SearchResult(0, exp_ctr.get_value());
    }

    /*
    namespace details {
        const int INT_INF = std::numeric_limits<int>::max();

        // TODO: examine this function. Ideally, RBFS should work better than IDA*,
        // but in my sliding block puzzle case RBFS contains so many more complex 
        // basic operations that IDA* wins out 
        template <class Puzzle, class Action, class HeuristicFunc>
            std::pair<bool, int> rbfs(const SearchNode<Puzzle> &node, int cost_limit, HeuristicFunc hf=HeuristicFunc()) {
                const Puzzle &p = node.entry;
                if(p.is_solved())
                    return std::make_pair(true, node.path_cost);
                std::vector<SearchNode<Puzzle>> successors;
                for(auto action : p.template possible_actions<Action>()) {
                    Puzzle new_p = p;
                    int new_path_cost = node.path_cost + new_p.apply_action(action);
                    int new_est_cost = new_path_cost + HeuristicFunc()(new_p);
                    successors.emplace_back(new_p, new_path_cost, new_est_cost);
                }
                if(successors.empty())
                    return std::make_pair(false, INT_INF); // failure
                for(auto &s : successors) // re-propagate cost if the path was previously found
                    s.est_cost = std::max(s.est_cost, node.est_cost);
                // now, we need to find the best/second best etc. multiple times, so I thought
                // it best to use a heap for the general case
                std::make_heap(successors.begin(), successors.end(), SearchNodeComparator<Puzzle>());
                while(true) {
                    // the best node is simply the first in the heap
                    const SearchNode<Puzzle> &best = successors[0];
                    if(best.est_cost > cost_limit)
                        return std::make_pair(false, best.est_cost);
                    // to find the alternative, pop the first and place it at the back
                    std::pop_heap(successors.begin(), successors.end(), SearchNodeComparator<Puzzle>());
                    int alt_cost = successors[0].est_cost;
                    // get a new reference for the best node, which is now at the back
                    SearchNode<Puzzle> &best2 = successors.back();
                    bool success = false;
                    std::tie(success, best2.est_cost) = rbfs<Puzzle, Action, HeuristicFunc>(best2, std::min(cost_limit, alt_cost), hf);
                    if(success)
                        return std::make_pair(true, best2.est_cost);
                    // since the cost of the best was modified, reinsert it into successors
                    std::push_heap(successors.begin(), successors.end(), SearchNodeComparator<Puzzle>());
                }
            }
    }

    template <class Puzzle, class Action, class HeuristicFunc>
        int recursive_best_first_search(const Puzzle &p) {
            using details::rbfs;
            using details::INT_INF;
            SearchNode<Puzzle> start_node(p, 0, HeuristicFunc()(p));
            bool success;
            int cost;
            std::tie(success, cost) = rbfs<Puzzle, Action, HeuristicFunc>(start_node, INT_INF);
            if(!success)
                details::throw_unreachable();
            return cost;
        }
    */

    template <class Puzzle, class HF>
    using SearchFunction = std::function<SearchResult(const Puzzle &, double, size_t, HF)>;

    template <class P, class A, class HF, class C = default_cost_t>
    SearchFunction<P, HF> search_factory(SearchType type) {
        typedef SearchType T;
        switch(type) {
            case T::BFS:                    return [](const P &p, double w, size_t b, HF hf) {
                                                return breadth_first_search<P, A, C>(p);
                                            };
//            case T::IDDFS:                  return [](const P &p, double w, size_t b, HF hf) {
//                                                return iterative_deepening_dfs<P, A, C>(p);
//                                            };
            case T::ASTAR:                  return [](const P &p, double w, size_t b, HF hf) {
                                                return a_star_search<P, A, HF, C>(p, hf);
                                            };
            case T::ID_ASTAR:               return [](const P &p, double w, size_t b, HF hf) {
                                                return iterative_deepening_a_star<P, A, HF, C>
                                                       (p, hf);
                                            };
            case T::WEIGHTED_ASTAR:         return [](const P &p, double w, size_t b, HF hf) {
                                                return weighted_a_star_search<P, A, HF, C>
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
