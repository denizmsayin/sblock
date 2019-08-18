#ifndef __SEARCH_HPP__
#define __SEARCH_HPP__

#include <algorithm>
#include <vector>
#include <list>
#include <unordered_set>
#include <unordered_map>
#include <stdexcept>

#include "search_queues.hpp"

// The class Puzzle has to implement the following functions:
//     bool is_solved() const { returns true if the Puzzle is in a goal state, false o.w. }
//     Container possible_actions() const { returns possible actions for the current puzzle }
//     int apply_action(Action a) { applies 'a' to the puzzle and returns the cost of the action }
// the class must also define a hash function so that it can be stored in unordered_set.
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
// Examples: GraphSearchIterator<Puzzle, Action, Zero Returning Functor, Queue> would be breadth first
//           GraphSearchIterator<Puzzle, Action, Zero Returning Functor, Stack> would be depth first
//           GraphSearchIterator<Puzzle, Action, Some Heuristic, PriorityQueue> would be A*
template <class Puzzle, class Action, class HeuristicFunc, template<class, class> class Queue>
class GraphSearchIterator {
public:
    struct Record {
        Puzzle state;
        Record *prev;
        Action last_action;
        int path_cost;
        int est_cost;

        Record(Puzzle p, Record *r, Action a, int pc, int ec) : state(p), prev(r), 
            last_action(a), path_cost(pc), est_cost(ec) {}
    };

    class RecordComparator {
    public:
        bool operator()(const Record *r1, const Record *r2) {
            return r1->est_cost > r2->est_cost;
        }
    };

    GraphSearchIterator(const Puzzle &initial_state, const Action &invalid_action);

    bool done() const;

    Record *next();

private:
    Queue<Record *, RecordComparator> frontier;
    std::list<Record> records;
    std::unordered_set<Puzzle> visited;
};


template <class P, class A, class HF, template<class, class> class Q>
GraphSearchIterator<P, A, HF, Q>
::GraphSearchIterator(const P &initial_state, const A &invalid_action) 
    : frontier(), records(), visited()
{
    records.emplace_back(
        initial_state, 
        nullptr, 
        invalid_action, 
        0, 
        HF()(initial_state)
    );
    frontier.push(&records.back());
}

template <class P, class A, class HF, template<class, class> class Q>
bool GraphSearchIterator<P, A, HF, Q>
::done() const 
{
    return frontier.empty();
}

template <class P, class A, class HF, template<class, class> class Q>
typename GraphSearchIterator<P, A, HF, Q>::Record*
GraphSearchIterator<P, A, HF, Q>
::next()
{
    // since we do not have a decrease-key operation, it is possible
    // to pull already visited states from the frontier, which is why
    // we need a loop to skip those states first
    Record *rec;
    do {
        rec = frontier.top(); // get the record from the top of the frontier
        frontier.pop();
    } while(visited.find(rec->state) != visited.end());
    // at this point we have reached a visited state
    visited.insert(rec->state); // add it to the visited state
    for(const auto &action : rec->state.possible_actions()) { // get the set of possible actions
        // copy the existing state, and mutate it + get its path cost
        P new_state = rec->state;
        int path_cost = new_state.apply_action(action);
        if(visited.find(new_state) == visited.end()) { // the new state is not visited
            int new_path_cost = rec->path_cost + path_cost; // g(n) in A*
            int est_cost = new_path_cost + HF()(new_state); // f(n) = g(n) + h(n)
            records.emplace_back(
                new_state, // the new state
                rec, // previous record leading to this one
                action, // action that led to this state
                new_path_cost, // path cost so far
                est_cost // estimated cost for the function
            );
            frontier.push(&records.back());
        }
    }
    return rec;
}

// A generic class that can be used to omit heuristics, always returns zero
template <class P>
class NoHeuristic {
public:
    int operator()(const P &p) {
        return 0;
    }
};

// Reconstruct the chain of actions that led to the provided record by following the pointers
template <class P, class A, class HF, template<class, class> class Q>
static std::vector<A> construct_action_chain(
        const typename GraphSearchIterator<P, A, HF, Q>::Record *rec) 
{
    std::vector<A> actions;
    for(auto rec_itr = rec; rec_itr->prev != nullptr; rec_itr = rec_itr->prev)
        actions.push_back(rec_itr->last_action);
    return actions;
}

// Reconstruct the chain of actions that led to the provided record by following the pointers, but reverse it at the end
template <class P, class A, class HF, template<class, class> class Q>
static std::vector<A> construct_reverse_action_chain(
        const typename GraphSearchIterator<P, A, HF, Q>::Record *rec) 
{
    std::vector<A> actions = construct_action_chain<P, A, HF, Q>(rec);
    std::reverse(actions.begin(), actions.end());
    return actions;
}

template <class P, class A, class HF, template<class, class> class Q>
static std::vector<A> construct_inverse_action_chain(
        const typename GraphSearchIterator<P, A, HF, Q>::Record *rec) 
{
    std::vector<A> actions;
    for(auto rec_itr = rec; rec_itr->prev != nullptr; rec_itr = rec_itr->prev)
        actions.push_back(inverse(rec_itr->last_action));
    return actions;
}


// a generic graph search, forms base for BFS, DFS, A*
template <class P, class A, class HF, template<class, class> class Q>
std::vector<A> graph_search(const P &initial_state, const A &invalid_action) {
    auto itr = GraphSearchIterator<P, A, HF, Q>(initial_state, invalid_action);
    while(!itr.done()) {
        auto rec = itr.next();
        if(rec->state.is_solved())
            // Reconstruct the set of actions from the record
            return construct_reverse_action_chain<P, A, HF, Q>(rec);
    }
    throw std::invalid_argument("The initial state cannot be transformed to the goal state");
    return std::vector<A>(); // to prevent void warning
}

template <class P, class A>
std::vector<A> depth_first_search(const P &initial_state, const A &invalid_action) {
    return graph_search<P, A, NoHeuristic<P>, Stack>(initial_state, invalid_action);
}

template <class P, class A>
std::vector<A> breadth_first_search(const P &initial_state, const A &invalid_action) {
    return graph_search<P, A, NoHeuristic<P>, Queue>(initial_state, invalid_action);
}

template <class P, class A, class HF>
std::vector<A> a_star_search(const P &initial_state, const A &invalid_action) {
    return graph_search<P, A, HF, PriorityQueue>(initial_state, invalid_action);
}

// Helper for bidirectional search
template <class P, class A, class HF, template<class, class> class Q>
bool process_itr(
    bool is_forward_itr,
    GraphSearchIterator<P, A, HF, Q> &itr, 
    std::unordered_map<P, typename GraphSearchIterator<P, A, HF, Q>::Record *> &visited,
    std::vector<A> &moves)
{
    if(!itr.done()) {
        auto rec = itr.next(); // get the next state
        auto stored_rec_itr = visited.find(rec->state); // try to find if it has already been reached
        if(stored_rec_itr != visited.end()) { // state has already been reached
            // decide on which chain is which chain depending on the iterator type
            auto f_rec = is_forward_itr ? rec : stored_rec_itr->second;
            auto b_rec = is_forward_itr ? stored_rec_itr->second : rec;
            // combine forward and backward chains and store them in 'moves'
            moves = construct_reverse_action_chain<P, A, HF, Q>(f_rec); // forward chain
            std::vector<A> ba = construct_inverse_action_chain<P, A, HF, Q>(b_rec); // backward chain
            moves.reserve(moves.size() + ba.size());
            moves.insert(moves.end(), ba.begin(), ba.end());
            return true;
        }
        // could not find a match, add to visited set and continue
        visited.insert(std::make_pair(rec->state, rec));
    }
    return false;
}


template <class P, class A, class HF, template<class, class> class Q>
std::vector<A> bidirectional_search(const P &initial_state, const A &invalid_action) {
    std::vector<A> moves;
    P goal = initial_state.goal_state();
    auto forward_itr = GraphSearchIterator<P, A, HF, Q>(initial_state, invalid_action);
    auto backward_itr = GraphSearchIterator<P, A, HF, Q>(goal, invalid_action);
    std::unordered_map<P, typename GraphSearchIterator<P, A, HF, Q>::Record *> visited;
    while(!forward_itr.done() && !backward_itr.done()) {
        // try the forward iterator first
        bool found = process_itr(true, forward_itr, visited, moves);
        // if not, the backward iterator
        if(!found)
            found = process_itr(false, backward_itr, visited, moves);
        // if the solution was found, return the moves
        if(found) 
            return moves;
    }
    throw std::invalid_argument("The initial state cannot be transformed to the goal state");
    return std::vector<A>(); // to prevent void warning
}

template <class P, class A>
std::vector<A> bidirectional_bfs(const P &initial_state, const A &invalid_action) {
    return bidirectional_search<P, A, NoHeuristic<P>, Queue>(initial_state, invalid_action);
}

#endif
