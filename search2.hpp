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

#include "search_queues.hpp"

// While the goal of achieving maximal code reuse was accomplished in the previous version
// of search.hpp, some algorithms suffered due to search being a high performance task,
// and lots of unnecessary operations being done. In this second header I will try to do a retake
// where almost every search function is implemented separately in order to maximize the performance
// obtained from each of them
//
// A base class for both tree and graph search classes
// The class Puzzle has to implement the following functions:
//     bool is_solved() const { returns true if the Puzzle is in a goal state, false o.w. }
//     Container possible_actions() const { returns possible actions for the current puzzle }
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

template <class Action>
struct BfsActionRecord {
    std::shared_ptr<BfsActionRecord> prev;
    Action action;

    BfsActionRecord(const std::shared_ptr<BfsActionRecord> &pp, const Action &a) : 
        prev(pp), action(a) {}
};

template <class Puzzle, class Action>
struct BfsNode {
    std::shared_ptr<BfsActionRecord<Action>> prev;
    Puzzle puzzle;

    BfsNode(const std::shared_ptr<BfsActionRecord<Action>> &pp, const Puzzle &p) : 
        prev(pp), puzzle(p) {}
    BfsNode(BfsActionRecord<Action> *pp, const Puzzle &p) : 
        prev(pp), puzzle(p) {}
};

template <class Puzzle, class Action>
std::vector<Action> reconstruct_action_chain(const BfsNode<Puzzle, Action> &node) {
    // first put all of the actions in a stack for reversing
    std::stack<Action> s;
    for(const BfsActionRecord<Action> *rec = node.prev.get(); rec != nullptr; rec = rec->prev.get())
        s.push(rec->action);
    // then pop them from the stack and write to output iterator
    std::vector<Action> v;
    while(!s.empty()) {
        v.push_back(s.top());
        s.pop();
    }
    return v;
}
}

// SEARCH NODE, the class that defines a search node, a record structure that keeps
// both a puzzle state and extra book-keeping information about it. Also holds some
// static variables such as a counter for the number of created nodes.
namespace details {
template <class Puzzle>
struct SearchNode {
    static size_t NODE_COUNTER;
    static size_t LAST_RECORDED_NC;
    static std::chrono::time_point<std::chrono::high_resolution_clock> LAST_RECORDED_T;

    Puzzle puzzle;
    int path_cost;
    int est_cost;

    SearchNode(const Puzzle &p, int pc, int ec) : puzzle(p), path_cost(pc), est_cost(ec) 
    {
        NODE_COUNTER++;
        if(NODE_COUNTER == 1) {
            record();
        }
        if(NODE_COUNTER % 100000000 == 0) {
            display();
            record();
        }
    }

    SearchNode(const Puzzle &p, int pc) : puzzle(p), path_cost(pc), est_cost(0) 
    {
        NODE_COUNTER++;
    }

private:
    void record() {
        LAST_RECORDED_NC = NODE_COUNTER;
        LAST_RECORDED_T = std::chrono::high_resolution_clock::now();
    }

    void display() {
        auto current_t = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms = current_t - LAST_RECORDED_T;
        size_t nps = static_cast<size_t>(1000.0 * (NODE_COUNTER - LAST_RECORDED_NC) / fp_ms.count());
        std::cout << "Node count: " << NODE_COUNTER/1000000 << "M, nps: " << nps << std::endl;
    }
};

template <class Puzzle>
size_t SearchNode<Puzzle>::NODE_COUNTER = 0;

template <class Puzzle>
size_t SearchNode<Puzzle>::LAST_RECORDED_NC = 0;

template <class Puzzle>
std::chrono::time_point<std::chrono::high_resolution_clock> SearchNode<Puzzle>::LAST_RECORDED_T = std::chrono::high_resolution_clock::now();

template <class Puzzle>
class SearchNodeComparator {
public:
    bool operator()(const SearchNode<Puzzle> &n1, const SearchNode<Puzzle> &n2) {
        return n1.est_cost > n2.est_cost;
    }
};
}

template <class Puzzle>
size_t get_node_counter() { return details::SearchNode<Puzzle>::NODE_COUNTER; }

template <class Puzzle>
void reset_node_counter() { details::SearchNode<Puzzle>::NODE_COUNTER = 0; }

namespace path_remembering {
template <class Puzzle, class Action>
std::vector<Action> breadth_first_search(const Puzzle &p) {
    std::unordered_set<Puzzle> visited;
    std::queue<details::BfsNode<Puzzle, Action>> q;
    q.emplace(nullptr, p);
    while(!q.empty()) {
        auto node = q.front(); q.pop(); // remove the top state
        const auto &p = node.puzzle;
        visited.insert(p);
        if(p.is_solved()) // if the puzzle is solved, return
            return reconstruct_action_chain(node);
        // otherwise, we have to expand each action into a new node
        for(auto action : p.possible_actions()) {
            Puzzle new_p = p; new_p.apply_action(action);
            if(visited.find(new_p) == visited.end()) // not visited
                q.emplace(new details::BfsActionRecord<Action>(node.prev, action), new_p);
        }
    }
    details::throw_unreachable();
}
}

template <class Puzzle, class Action>
int breadth_first_search(const Puzzle &p) {
    using details::SearchNode;
    std::unordered_set<Puzzle> visited;
    std::queue<SearchNode<Puzzle>> q;
    q.emplace(p, 0);
    while(!q.empty()) {
        auto node = q.front(); q.pop(); // remove the top state
        const auto &p = node.puzzle;
        visited.insert(p);
        if(p.is_solved()) // if the puzzle is solved, return
            return node.path_cost;
        // otherwise, we have to expand each action into a new node
        for(auto action : p.possible_actions()) {
            Puzzle new_p = p; 
            int path_cost = new_p.apply_action(action);
            if(visited.find(new_p) == visited.end()) // not visited
                q.emplace(new_p, node.path_cost + path_cost);
        }
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
        for(auto action : p.possible_actions()) {
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
int step_single_direction(std::queue<details::SearchNode<Puzzle>> &queue,
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
        for(auto action : p.possible_actions()) {
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
    using details::SearchNode;
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
int a_star_search(const Puzzle &p) {
    using details::SearchNode;
    using details::SearchNodeComparator;
    std::unordered_set<Puzzle> visited;
    std::priority_queue<SearchNode<Puzzle>, std::vector<SearchNode<Puzzle>>, SearchNodeComparator<Puzzle>> pq;
    pq.emplace(p, 0, HeuristicFunc()(p));
    while(!pq.empty()) {
        auto node = pq.top(); pq.pop();
        const auto &p = node.puzzle;
        if(p.is_solved())
            return node.path_cost;
        visited.insert(p);
        for(auto action : p.possible_actions()) {
            Puzzle new_p = p;
            int step_cost = new_p.apply_action(action);
            if(visited.find(new_p) == visited.end()) {
                int new_path_cost = node.path_cost + step_cost;
                int new_est_cost = new_path_cost + HeuristicFunc()(new_p);
                pq.emplace(new_p, new_path_cost, new_est_cost);
            }
        }
    }
    details::throw_unreachable();
    return 0;
}

namespace details {

// TODO: this implementation of IDA* cannot decide on NOT_FOUND
template <class Puzzle, class Action, class HeuristicFunc>
int cost_limited_dfs(const SearchNode<Puzzle> &node, int cost_limit) {
    if(node.est_cost > cost_limit)
        return node.est_cost;
    else if(node.puzzle.is_solved()) {
        return node.path_cost;;
    } else {
        int min_exceeding_cost = std::numeric_limits<int>::max();
        for(auto action : node.puzzle.possible_actions()) {
            Puzzle new_p = node.puzzle; 
            int new_path_cost = node.path_cost + new_p.apply_action(action);
            int new_est_cost = new_path_cost + HeuristicFunc()(new_p);
            SearchNode<Puzzle> new_node(new_p, new_path_cost, new_est_cost);
            int result = cost_limited_dfs<Puzzle, Action, HeuristicFunc>(new_node, cost_limit);
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
int iterative_deepening_a_star(const Puzzle &p) {
    using details::SearchNode;
    using details::cost_limited_dfs;
    int cost_limit = HeuristicFunc()(p);
    SearchNode<Puzzle> start_node(p, 0, cost_limit);
    while(true) {
        std::cout << "New cost limit: " << cost_limit << std::endl;
        int result = details::cost_limited_dfs<Puzzle, Action, HeuristicFunc>(start_node, cost_limit);
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
std::pair<bool, int> rbfs(const SearchNode<Puzzle> &node, int cost_limit) {
    const Puzzle &p = node.puzzle;
    if(p.is_solved())
        return std::make_pair(true, node.path_cost);
    std::vector<SearchNode<Puzzle>> successors;
    for(auto action : p.possible_actions()) {
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
        std::tie(success, best2.est_cost) = rbfs<Puzzle, Action, HeuristicFunc>(best2, std::min(cost_limit, alt_cost));
        if(success)
            return std::make_pair(true, best2.est_cost);
        // since the cost of the best was modified, reinsert it into successors
        std::push_heap(successors.begin(), successors.end(), SearchNodeComparator<Puzzle>());
    }
}
}

template <class Puzzle, class Action, class HeuristicFunc>
int recursive_best_first_search(const Puzzle &p) {
    using details::SearchNode;
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


}
#endif
