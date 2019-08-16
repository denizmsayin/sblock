#ifndef __SEARCH_HPP__
#define __SEARCH_HPP__

#include <algorithm>
#include <vector>
#include <list>
#include <unordered_set>
#include <stdexcept>

template <class Puzzle, class Action, class ActionFunc, class SuccessorFunc, class HeuristicFunc, class Queue>
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

    GraphSearchIterator(const Puzzle &initial_state, const Action &invalid_function);

    bool done() const;

    Record next();

private:
    Queue frontier;
    std::list<Record> records;
    std::unordered_set<Puzzle> visited;
};

template <class P, class A, class AF, class SF, class HF, class Q>
class RecordComparator {
using Rec = typename GraphSearchIterator<P, A, AF, SF, HF, Q>::Record;
public:
    bool operator()(const Rec &r1, const Rec &r2) {
        return r1.est_cost > r2.est_cost;
    }
};

template <class P, class A, class AF, class SF, class HF, class Q>
GraphSearchIterator<P, A, AF, SF, HF, Q>
::GraphSearchIterator(const P &initial_state, const A &invalid_action) 
{
    frontier = Q(); // TODO: move to initializer list
    records.emplace_back(
        initial_state, 
        nullptr, 
        invalid_action, 
        0, 
        HF(initial_state)
    );
    frontier.push(&records.back());
}

template <class P, class A, class AF, class SF, class HF, class Q>
bool GraphSearchIterator<P, A, AF, SF, HF, Q>
::done() const 
{
    return frontier.empty();
}

template <class P, class A, class AF, class SF, class HF, class Q>
typename GraphSearchIterator<P, A, AF, SF, HF, Q>::Record
GraphSearchIterator<P, A, AF, SF, HF, Q>
::next()
{
    Record *rec = frontier.top(); // get the record from the top of the frontier
    if(visited.find(rec->state) == visited.end()) { // if the state in the record is unvisited
        visited.push(rec->state); // add it to the visited state
        for(auto &action : AF(rec->state)) { // get the set of possible actions
            // copy the existing state, and mutate it + get its path cost
            P new_state = rec->state;
            int path_cost = SF(new_state, action);
            if(visited.find(new_state) == visited.end()) { // the new state is not visited
                int new_path_cost = rec->path_cost + path_cost; // g(n) in A*
                int est_cost = new_path_cost + HF(new_state); // f(n) = g(n) + h(n)
                records.emplace_back(
                    new_state, // the new state
                    rec, // previous record leading to this one
                    action, // action that led to this state
                    new_path_cost, // path cost so far
                    est_cost // estimated cost for the function
                );
                frontier.push_back(&records.back());
            }
        }
    }
    return *rec;
}

template <class P, class A, class GoalTestFunc, class AF, class SF, class HF, class Q>
std::vector<A> graphSearch(const P &initial_state, const A &invalid_action) {
    auto itr = GraphSearchIterator<P, A, AF, SF, HF, Q>(initial_state, invalid_action);
    while(!itr.done()) {
        auto rec = itr.next();
        if(GoalTestFunc(rec.state)) {
            // Reconstruct the set of actions from the record
            std::vector<A> actions;
            for(auto rec_itr = &rec; rec_itr->prev != nullptr; rec_itr = rec_itr->prev)
                actions.push_back(rec_itr->action);
            std::reverse(actions.begin(), actions.end());
            return actions;
        }
    }
    throw std::invalid_argument("The initial state cannot be transformed to the goal state");
    return std::vector<A>(); // to prevent void warning
}

#endif
