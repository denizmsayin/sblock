#ifndef __SEARCH_HPP__
#define __SEARCH_HPP__

#include <algorithm>
#include <vector>
#include <list>
#include <unordered_set>
#include <stdexcept>

// The class Puzzle has to implement the following functions:
//     bool is_solved() const { returns true if the Puzzle is in a goal state, false o.w. }
//     Container possible_actions() const { returns possible actions for the current puzzle }
//     int apply_action(Action a) { applies 'a' to the puzzle and returns the cost of the action }
// the class must also define a hash function so that it can be stored in unordered_set.
//
// The class Action must be used and returned by Puzzle as explained above.
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

    GraphSearchIterator(const Puzzle &initial_state, const Action &invalid_function);

    bool done() const;

    Record next();

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
typename GraphSearchIterator<P, A, HF, Q>::Record
GraphSearchIterator<P, A, HF, Q>
::next()
{
    Record *rec = frontier.top(); // get the record from the top of the frontier
    frontier.pop();
    if(visited.find(rec->state) == visited.end()) { // if the state in the record is unvisited
        visited.insert(rec->state); // add it to the visited state
        for(auto &action : rec->state.possible_actions()) { // get the set of possible actions
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
    }
    return *rec;
}

template <class P, class A, class HF, template<class, class> class Q>
std::vector<A> graphSearch(const P &initial_state, const A &invalid_action) {
    auto itr = GraphSearchIterator<P, A, HF, Q>(initial_state, invalid_action);
    while(!itr.done()) {
        auto rec = itr.next();
        if(rec.state.is_solved()) {
            // Reconstruct the set of actions from the record
            std::vector<A> actions;
            for(auto rec_itr = &rec; rec_itr->prev != nullptr; rec_itr = rec_itr->prev)
                actions.push_back(rec_itr->last_action);
            std::reverse(actions.begin(), actions.end());
            return actions;
        }
    }
    throw std::invalid_argument("The initial state cannot be transformed to the goal state");
    return std::vector<A>(); // to prevent void warning
}

#endif
