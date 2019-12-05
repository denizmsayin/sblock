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

#include "sblock_utils.hpp"
#include "search_queues.hpp"

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
namespace details {
void throw_unreachable() {
    throw std::invalid_argument("The goal state is not reachable from the provided initial state");
}
}

// SEARCH NODE, the class that defines a search node, a record structure that keeps
// both a puzzle state and extra book-keeping information about it. Also holds some
// static variables such as a counter for the number of created nodes.
template <class Puzzle>
struct SearchNode {
    static size_t NODE_COUNTER;

    #ifdef TRACK_NODES
    static SeriesTracker<size_t> COUNTER_TRACKER;
    #endif

    Puzzle puzzle;
    uint8_t path_cost;
    uint8_t est_cost;

    SearchNode(const Puzzle &p, int pc, int ec) : puzzle(p), path_cost(pc), est_cost(ec) 
    {
        NODE_COUNTER++;
        #ifdef TRACK_NODES
        COUNTER_TRACKER.track();
        #endif
    }

    SearchNode(const Puzzle &p, int pc) : puzzle(p), path_cost(pc), est_cost(0) 
    {
        NODE_COUNTER++;
        #ifdef TRACK_NODES
        COUNTER_TRACKER.track();
        #endif
    }
};

template <class Puzzle>
size_t SearchNode<Puzzle>::NODE_COUNTER = 0;

#ifdef TRACK_NODES
template <class Puzzle>
SeriesTracker<size_t> SearchNode<Puzzle>::COUNTER_TRACKER(&SearchNode<Puzzle>::NODE_COUNTER, SeriesTracker<size_t>::Options(10000000));
#endif

template <class Puzzle>
class SearchNodeComparator {
public:
    bool operator()(const SearchNode<Puzzle> &n1, const SearchNode<Puzzle> &n2) {
        return n1.est_cost > n2.est_cost;
    }
};

template <class Puzzle>
class RevSearchNodeComparator {
public:
    bool operator()(const SearchNode<Puzzle> &n1, const SearchNode<Puzzle> &n2) {
        return n1.est_cost < n2.est_cost;
    }
};

template <class Puzzle>
size_t get_node_counter() { return SearchNode<Puzzle>::NODE_COUNTER; }

template <class Puzzle>
void reset_node_counter() { SearchNode<Puzzle>::NODE_COUNTER = 0; }

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
    const Puzzle &p = node.puzzle;
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

template <class Puzzle, class Action>
int breadth_first_search(const Puzzle &p) {
    BreadthFirstIterator<Puzzle, Action> itr(p);
    while(!itr.done()) {
        SearchNode<Puzzle> node = itr.next();
        if(node.puzzle.is_solved())
            return node.path_cost;
    }
    details::throw_unreachable();
    return 0;
}

template <class Puzzle, class Action>
int bfs_double_q(const Puzzle &p) {
    std::queue<Puzzle> q1, q2;
    std::queue<Puzzle> *qf = &q1, *qn = &q2;
    std::unordered_set<Puzzle> v;
    int cost = 0;
    qf->emplace(p);
    v.emplace(p);
    while(!qf->empty()) {
        while(!qf->empty()) {
            Puzzle p = qf->front(); qf->pop();
            if(p.is_solved())
                return cost;
            for(const auto &action : p.template possible_actions<Action>()) {
                Puzzle new_p = p; new_p.apply_action(action);
                if(v.find(new_p) == v.end()) {
                    qn->emplace(new_p);
                    v.emplace(new_p);
                }
            }
        }
        cost +=1;
        std::swap(qf, qn);
    }
    details::throw_unreachable();
    return 0;
}

enum class IDFSResult {
    CUTOFF,
    FOUND,
    NOT_FOUND
};

template <class Puzzle, class Action>
IDFSResult depth_limited_dfs(const Puzzle &p, int depth) {
    if(p.is_solved()) {
        return IDFSResult::FOUND;
    } else if(depth == 0) {
        return IDFSResult::CUTOFF;
    } else {
        bool cutoff = false;
        for(auto action : p.template possible_actions<Action>()) {
            Puzzle new_p = p; new_p.apply_action(action);
            IDFSResult result = depth_limited_dfs<Puzzle, Action>(new_p, depth-1);
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
int iterative_deepening_dfs(const Puzzle &p) {
    static int LIMIT = 1000000;
    for(int depth = 0; depth < LIMIT; depth++) {
        IDFSResult result = depth_limited_dfs<Puzzle, Action>(p, depth);
        if(result == IDFSResult::FOUND)
            return depth;
        else if(result == IDFSResult::NOT_FOUND) {
            details::throw_unreachable();
            return 0;
        }
    }
    throw std::invalid_argument("The goal state could not be reached in 1M steps");
}


namespace details {
template <class Puzzle, class Action>
int step_single_direction(std::queue<SearchNode<Puzzle>> &queue,
                          std::unordered_map<Puzzle, int> &visited, 
                          std::unordered_map<Puzzle, int> &other_visited)
{
    if(!queue.empty()) {
        auto node = queue.front(); queue.pop();
        const auto &p = node.puzzle;
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

template <class Puzzle, class Action, class HeuristicFunc>
int a_star_search(const Puzzle &p, HeuristicFunc hf=HeuristicFunc()) {
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
    std::unordered_map<Puzzle, int> visited;
    std::priority_queue<SearchNode<Puzzle>, std::vector<SearchNode<Puzzle>>, SearchNodeComparator<Puzzle>> pq;
    pq.emplace(p, 0, hf(p));
    while(!pq.empty()) {
        auto node = pq.top(); pq.pop();
        const auto &p = node.puzzle;
        // check if the state has been visited before
        // we act as if not visited if the cost is smaller than the prev one too
        auto lookup = visited.find(p);
        if(lookup == visited.end() || node.path_cost < lookup->second) {
            if(p.is_solved())
                return node.path_cost;
            visited.emplace(p, node.path_cost);
            for(auto action : p.template possible_actions<Action>()) {
                Puzzle new_p = p;
                int step_cost = new_p.apply_action(action);
                int new_path_cost = node.path_cost + step_cost;
                auto lookup = visited.find(new_p);
                if(lookup == visited.end() || new_path_cost < lookup->second) {
                    int new_est_cost = new_path_cost + hf(new_p);
                    pq.emplace(new_p, new_path_cost, new_est_cost);
                }
            }
        }
    }
    details::throw_unreachable();
    return 0;
}

namespace details {

// TODO: this implementation of IDA* cannot decide on NOT_FOUND
template <class Puzzle, class Action, class HeuristicFunc>
int cost_limited_dfs(const SearchNode<Puzzle> &node, int cost_limit, HeuristicFunc hf=HeuristicFunc()) {
    if(node.est_cost > cost_limit)
        return node.est_cost;
    else if(node.puzzle.is_solved()) {
        return node.path_cost;;
    } else {
        int min_exceeding_cost = std::numeric_limits<int>::max();
        for(auto action : node.puzzle.template possible_actions<Action>()) {
            Puzzle new_p = node.puzzle; 
            int new_path_cost = node.path_cost + new_p.apply_action(action);
            int new_est_cost = new_path_cost + hf(new_p);
            SearchNode<Puzzle> new_node(new_p, new_path_cost, new_est_cost);
            int result = cost_limited_dfs<Puzzle, Action, HeuristicFunc>(new_node, cost_limit, hf);
            if(result <= cost_limit) // found
                return result;
            else if(min_exceeding_cost > result) // not found, but less than smallest exceeding
                min_exceeding_cost = result;
        }
        return min_exceeding_cost;
    }
}
}

template <class Puzzle, class Action, class HeuristicFunc>
int iterative_deepening_a_star(const Puzzle &p, HeuristicFunc hf=HeuristicFunc()) {
    using details::cost_limited_dfs;
    int cost_limit = hf(p);
    SearchNode<Puzzle> start_node(p, 0, cost_limit);
    while(true) {
        int result = details::cost_limited_dfs<Puzzle, Action, HeuristicFunc>(start_node, cost_limit, hf);
        if(result <= cost_limit) 
            return result;
        cost_limit = result;
    }
    return 0;
}

namespace details {
const int INT_INF = std::numeric_limits<int>::max();

// TODO: examine this function. Ideally, RBFS should work better than IDA*,
// but in my sliding block puzzle case RBFS contains so many more complex 
// basic operations that IDA* wins out 
template <class Puzzle, class Action, class HeuristicFunc>
std::pair<bool, int> rbfs(const SearchNode<Puzzle> &node, int cost_limit, HeuristicFunc hf=HeuristicFunc()) {
    const Puzzle &p = node.puzzle;
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

namespace details {

template <class Puzzle, class Action, class HeuristicFunc>
int cost_limited_dfs_nl(const SearchNode<Puzzle> &node, 
                     std::unordered_map<Puzzle, int> visited,
                     int cost_limit,
                     HeuristicFunc hf=HeuristicFunc()) {
    visited.emplace(node.puzzle, node.path_cost);
    if(node.est_cost > cost_limit)
        return node.est_cost;
    else if(node.puzzle.is_solved()) {
        return node.path_cost;;
    } else {
        int min_exceeding_cost = std::numeric_limits<int>::max();
        for(auto action : node.puzzle.template possible_actions<Action>()) {
            Puzzle new_p = node.puzzle; 
            int new_path_cost = node.path_cost + new_p.apply_action(action);
            auto lookup = visited.find(new_p);
            if(lookup == visited.end() || new_path_cost < lookup->second) {
                int new_est_cost = new_path_cost + HeuristicFunc()(new_p);
                SearchNode<Puzzle> new_node(new_p, new_path_cost, new_est_cost);
                int result = cost_limited_dfs_nl<Puzzle, Action, HeuristicFunc>(new_node, visited, cost_limit, hf);
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

template <class Puzzle, class Action, class HeuristicFunc>
int iterative_deepening_a_star_noloop(const Puzzle &p) {
    using details::cost_limited_dfs;
    int cost_limit = HeuristicFunc()(p);
    SearchNode<Puzzle> start_node(p, 0, cost_limit);
    while(true) {
        std::cout << "New cost limit: " << cost_limit << std::endl;
        std::unordered_map<Puzzle, int> visited;
        int result = details::cost_limited_dfs_nl<Puzzle, Action, HeuristicFunc>(start_node, visited, cost_limit);
        if(result <= cost_limit) 
            return result;
        cost_limit = result;
    }
    return 0;
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

namespace details {

template <class Puzzle, class Action, class HeuristicFunc>
int cost_limited_dfs_improved(const SearchNode<Puzzle> &node, 
                              std::unordered_map<Puzzle, int> &visited,
                              int cost_limit) 
{
    visited.emplace(node.puzzle, node.path_cost);
    if(node.est_cost > cost_limit)
        return node.est_cost;
    else if(node.puzzle.is_solved()) {
        return node.path_cost;;
    } else {
        int min_exceeding_cost = std::numeric_limits<int>::max();
        std::vector<SearchNode<Puzzle>> new_nodes;
        for(auto action : node.puzzle.template possible_actions<Action>()) {
            Puzzle new_p = node.puzzle; 
            int new_path_cost = node.path_cost + new_p.apply_action(action);
            auto lookup = visited.find(new_p);
            if(lookup == visited.end() || new_path_cost < lookup->second) {
                int new_est_cost = new_path_cost + HeuristicFunc()(new_p);
                new_nodes.emplace_back(new_p, new_path_cost, new_est_cost);
            }
        }
        // heapify the new nodes and try them one by one starting from the best first
        std::sort(new_nodes.begin(), new_nodes.end(), RevSearchNodeComparator<Puzzle>());
        for(const auto &new_node : new_nodes) {
            int result = cost_limited_dfs_improved<Puzzle, Action, HeuristicFunc>(new_node, visited, cost_limit);
            if(result <= cost_limit) // found
                return result;
            else if(min_exceeding_cost > result) // not found, but less than smallest exceeding
                min_exceeding_cost = result;
        }
        return min_exceeding_cost;
    }
}

}

template <class Puzzle, class Action, class HeuristicFunc>
int iterative_deepening_a_star_improved(const Puzzle &p) {
    using details::cost_limited_dfs;
    int cost_limit = HeuristicFunc()(p);
    SearchNode<Puzzle> start_node(p, 0, cost_limit);
    while(true) {
        std::unordered_map<Puzzle, int> visited;
        int result = details::cost_limited_dfs_improved<Puzzle, Action, HeuristicFunc>(start_node, visited, cost_limit);
        if(result <= cost_limit) 
            return result;
        cost_limit = result;
    }
    return 0;
}



}
#endif
