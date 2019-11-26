#ifndef __SEARCH_HPP__
#define __SEARCH_HPP__

#include <algorithm>
#include <memory>
#include <vector>
#include <list>
#include <unordered_set>
#include <unordered_map>
#include <stdexcept>
#include <limits>

#include "search_queues.hpp"

// The goal of this header is to provide all sorts of searching functions while achieving
// maximal code reuse, rather than maximal optimisation.

// A base class for both tree and graph search classes
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
// To increase code reuse, I decided to have both tree and graph search classes to inherit from
// the same class, and have visitation insertions/checks performed by virtual functions. Graph
// search does the insertions & checks in these functions, while the tree search simply skips
// them
//
// Examples: BaseSearchIterator<Puzzle, Action, Zero Returning Functor, Queue> would be breadth first
//           BaseSearchIterator<Puzzle, Action, Zero Returning Functor, Stack> would be depth first
//           BaseSearchIterator<Puzzle, Action, Some Heuristic, PriorityQueue> would be A*
template <class Puzzle, class Action, class HeuristicFunc, template<class, class> class Queue>
class BaseSearchIterator {
public:
    struct Record {
        Puzzle state;
        std::shared_ptr<Record> prev;
        Action last_action;
        int path_cost;
        int est_cost;

        Record(const Puzzle &p, Record *r, const Action &a, int pc, int ec) : state(p),
            prev(r), last_action(a), path_cost(pc), est_cost(ec) {}
        Record(const Puzzle &p, const std::shared_ptr<Record> &r, const Action &a, int pc, int ec)
            : state(p), prev(r), last_action(a), path_cost(pc), est_cost(ec) {}
    };

    class RecordComparator {
    public:
        bool operator()(const std::shared_ptr<Record> &r1, const std::shared_ptr<Record> &r2) {
            return r1->est_cost > r2->est_cost;
        }
    };

    BaseSearchIterator(const Puzzle &initial_state, const Action &invalid_action);
    BaseSearchIterator(const Puzzle &initial_state, const Action &invalid_action, int cost_lim);

    bool done() const;

    std::shared_ptr<Record> next();

    int get_exceeding_cost() const { return exceeding_cost; };

    constexpr static int MAX_COST = std::numeric_limits<int>::max();

protected:
    virtual bool is_visited(const Puzzle &state) const = 0;
    virtual void visited_insert(const Puzzle &state) = 0;

private:
    // A note on remembering paths:
    // Since one of the aims of this search library is being able to remember the path
    // to the solution, it requires the implementation of extra things that deal with
    // this issue. To prevent storing the whole path inside the record for each node,
    // I decided to implement a chain of links between records, with each record
    // pointing to the record which generated it, all the way to the root. Since records
    // are popped from the queue when they have been processed, they are lost, which
    // implies that we need to work out a way to preserve them.
    //
    // My initial idea was to store records in a global list, and store their pointers
    // in the queue. This worked well enough well enough for BFS and A*, but completely
    // eliminates the point of using depth-first approaches such as IDDFS and IDA*, due
    // to the records from cut-off paths being remembered.
    //
    // I have considered how to solve this, and decided to do away with the global list,
    // and instead dynamically allocate each record. Smart pointers to dynamically allocated
    // records will be stored both by the original entry in the queue, and by child states
    // still remaining in the queue in the case they are removed. Once all references are
    // gone, they will be automatically cleaned up by the smart pointer. In case too many
    // small allocations cause a significant performance hit, I will implement my own
    // object cache to handle their allocation in bulk, but only after profiling.

    // The next thing is that states are generated somewhat prematurely for DFS and its
    // derivatives, due to all successors being put on the stack, and not used until long
    // after. This causes the memory complexity to be O(bd), where b is the branching factor
    // and d the solution depth, instead of simply O(d). To remedy this, I plan to generate
    // successors one by one, and leave records in the frontier until all their successors
    // have been generated. This will require the records to store where they're left at in
    // the list of actions. It is possible that this will slightly slow down BFS, A* etc.,
    // but it should improve the memory complexity of IDDFS and IDA* nicely

    // LOG:
    // -----
    // -> Implemented with shared_ptrs, the program takes around 12.5% longer to terminate
    //    compared to previous commit, memory usage fell from approx. 2.5 GB to approx. 2 GB
    //    for the first instance of 4x4 with seed 42
    // -> I tried the idea of generating successors one by one, the results were not much
    //    better than before, shelved for now
    // -> I changed SBPuzzle's height and width to template parameters so that they
    //    become compile time constants, leaving only a few uint8_t's as members. Memory
    //    usage fell from around 2 GB to 800 MB for the first instance of 4x4, and time
    //    dropped from around 27 sec to 20 sec

    Queue<std::shared_ptr<Record>, RecordComparator> frontier;
    int cost_limit;
    int exceeding_cost;

    void init(const Puzzle &initial_state, const Action &invalid_action);
    void clean_frontier();
};

// the tree search iterator does not keep track of visited states and may revisit them
template <class P, class A, class HF, template<class, class> class Q>
class TreeSearchIterator : public BaseSearchIterator<P, A, HF, Q> {
public:
    TreeSearchIterator(const P &is, const A &ia) : BaseSearchIterator<P, A, HF, Q>(is, ia) {}
    TreeSearchIterator(const P &is, const A &ia, int cl) :
        BaseSearchIterator<P, A, HF, Q>(is, ia, cl) {}
protected:
    virtual bool is_visited(const P &state) const { return false; }
    virtual void visited_insert(const P &state) {}
};

// graph search keeps track of visited states and prevents them from being revisited
template <class P, class A, class HF, template<class, class> class Q>
class GraphSearchIterator : public BaseSearchIterator<P, A, HF, Q> {
public:
    GraphSearchIterator(const P &is, const A &ia) : BaseSearchIterator<P, A, HF, Q>(is, ia),
        visited() {}
    GraphSearchIterator(const P &is, const A &ia, int cl) :
        BaseSearchIterator<P, A, HF, Q>(is, ia, cl), visited() {}

protected:
    virtual bool is_visited(const P &state) const {
        return visited.find(state) != visited.end();
    }
    virtual void visited_insert(const P &state) {
        visited.insert(state);
    }

private:
    std::unordered_set<P> visited;
};

template <class P, class A, class HF, template <class, class> class Q>
void BaseSearchIterator<P, A, HF, Q>::init(const P &initial_state, const A &invalid_action)
{
    frontier.emplace(new Record(
        initial_state,
        nullptr,
        invalid_action,
        0,
        HF()(initial_state)
    ));
}

// Since we are constructing an iterator, which checks to see if the frontier
// is empty to see if it is done, we should always keep the frontier clean of
// unacceptable states at its top before the next call of .next()! This is
// done by removing visited and cost exceeding states from the top. Note
// that we also have to keep track of the minimum cost that exceeds the
// limit
template <class P, class A, class HF, template <class, class> class Q>
void BaseSearchIterator<P, A, HF, Q>::clean_frontier() {
    while(!frontier.empty()
          && (is_visited(frontier.top()->state)
              || frontier.top()->est_cost > cost_limit))
    {
        int est_cost = frontier.top()->est_cost;
        if(est_cost > cost_limit && est_cost < exceeding_cost)
            exceeding_cost = est_cost;
        frontier.pop();
    }
}

template <class P, class A, class HF, template<class, class> class Q>
BaseSearchIterator<P, A, HF, Q>
::BaseSearchIterator(const P &initial_state, const A &invalid_action)
    : frontier(), cost_limit(MAX_COST), exceeding_cost(MAX_COST)
{
    init(initial_state, invalid_action);
}

template <class P, class A, class HF, template<class, class> class Q>
BaseSearchIterator<P, A, HF, Q>
::BaseSearchIterator(const P &initial_state, const A &invalid_action, int cost_limit)
    : frontier(), cost_limit(cost_limit), exceeding_cost(MAX_COST)
{
    init(initial_state, invalid_action);
}

template <class P, class A, class HF, template<class, class> class Q>
bool BaseSearchIterator<P, A, HF, Q>
::done() const
{
    return frontier.empty();
}

// Returns the next valid state with its record
template <class P, class A, class HF, template<class, class> class Q>
std::shared_ptr<typename BaseSearchIterator<P, A, HF, Q>::Record>
BaseSearchIterator<P, A, HF, Q>
::next()
{
    // assumption: the state we take from the frontier is unvisited
    std::shared_ptr<Record> rec = frontier.top();
    frontier.pop();
    visited_insert(rec->state); // add it to the visited states
    for(const auto &action : rec->state.possible_actions()) { // get the set of possible actions
        // copy the existing state, and mutate it + get its path cost
        P new_state = rec->state;
        int path_cost = new_state.apply_action(action);
        if(!is_visited(new_state)) { // the new state is not visited
            int new_path_cost = rec->path_cost + path_cost; // g(n) in A*
            int est_cost = new_path_cost + HF()(new_state); // f(n) = g(n) + h(n)
            if(est_cost <= cost_limit) { // if the state does not exceed the cost limit
                frontier.emplace(new Record(
                    new_state, // the new state
                    rec, // previous record leading to this one
                    action, // action that led to this state
                    new_path_cost, // path cost so far
                    est_cost // estimated cost for the function
                ));
            } else if(est_cost < exceeding_cost) {
                exceeding_cost = est_cost;
            }
        }
    }
    clean_frontier();
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
        const std::shared_ptr<typename BaseSearchIterator<P, A, HF, Q>::Record> &rec)
{
    std::vector<A> actions;
    for(auto rec_itr = rec; rec_itr->prev != nullptr; rec_itr = rec_itr->prev)
        actions.push_back(rec_itr->last_action);
    return actions;
}

// Reconstruct the chain of actions that led to the provided record by following the pointers, but reverse it at the end
template <class P, class A, class HF, template<class, class> class Q>
static std::vector<A> construct_reverse_action_chain(
        const std::shared_ptr<typename BaseSearchIterator<P, A, HF, Q>::Record> &rec)
{
    std::vector<A> actions = construct_action_chain<P, A, HF, Q>(rec);
    std::reverse(actions.begin(), actions.end());
    return actions;
}

template <class P, class A, class HF, template<class, class> class Q>
static std::vector<A> construct_inverse_action_chain(
        const std::shared_ptr<typename BaseSearchIterator<P, A, HF, Q>::Record> &rec)
{
    std::vector<A> actions;
    for(auto rec_itr = rec; rec_itr->prev != nullptr; rec_itr = rec_itr->prev)
        actions.push_back(inverse(rec_itr->last_action));
    return actions;
}

// a generic graph search, forms base for BFS, DFS, A*, IDDFS, IDA* etc.
template <class P, class A, class HF, template<class, class> class Q,
          template <class, class, class, template <class, class> class> class Iterator>
std::vector<A> deepening_search(
    const P &initial_state,
    const A &invalid_action,
    int initial_max_cost)
{
    // start with an initial max cost
    int max_cost = initial_max_cost;
    while(true) {
        auto itr = Iterator<P, A, HF, Q>(initial_state, invalid_action, max_cost);
        // go through states returned by the iterator until there are none left
        while(!itr.done()) {
            auto rec = itr.next();
            if(rec->state.is_solved())
                // Reconstruct the set of actions from the record
                return construct_reverse_action_chain<P, A, HF, Q>(rec);
        }
        // at this point, either all states have been explored or the cost exceeded
        if(itr.get_exceeding_cost() == Iterator<P, A, HF, Q>::MAX_COST) // none exceeded
            break;
        max_cost = itr.get_exceeding_cost(); // some exceeded, get the next cost
    }
    throw std::invalid_argument("The initial state cannot be transformed to the goal state");
    return std::vector<A>(); // to prevent void warning
}

// Helper for bidirectional search
template <class P, class A, class HF, template<class, class> class Q,
          template <class, class, class, template<class, class> class> class Iterator>
static bool process_itr(
    bool is_forward_itr,
    Iterator<P, A, HF, Q> &itr,
    std::unordered_map<P, std::shared_ptr<typename BaseSearchIterator<P, A, HF, Q>::Record>> &visited,
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

// Bidirectional search, which works by intersecting states reached from the starting
// point and goal by advancing both at the same time
template <class P, class A, class HF, template<class, class> class Q,
          template <class, class, class, template<class, class> class> class Iterator>
std::vector<A> bidirectional_search(const P &initial_state, const A &invalid_action) {
    std::vector<A> moves;
    P goal = initial_state.goal_state();
    auto forward_itr = Iterator<P, A, HF, Q>(initial_state, invalid_action);
    auto backward_itr = Iterator<P, A, HF, Q>(goal, invalid_action);
    std::unordered_map<P, std::shared_ptr<typename BaseSearchIterator<P, A, HF, Q>::Record>> visited;
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

// From here on out, there are lots of derived search functions which form a hierarchy,
// with leaves being the usual DFS, BFS, A* etc.

template <class P, class A, class HF, template<class, class> class Q>
std::vector<A> deepening_tree_search(
    const P &initial_state,
    const A &invalid_action,
    int initial_max_cost)
{
    return deepening_search<P, A, HF, Q, TreeSearchIterator>(initial_state, invalid_action, initial_max_cost);
}

template <class P, class A, class HF, template<class, class> class Q>
std::vector<A> deepening_graph_search(
    const P &initial_state,
    const A &invalid_action,
    int initial_max_cost)
{
    return deepening_search<P, A, HF, Q, GraphSearchIterator>(initial_state, invalid_action, initial_max_cost);
}

template <class P, class A>
std::vector<A> iterative_deepening_dfs(const P &initial_state, const A &invalid_action) {
    return deepening_tree_search<P, A, NoHeuristic<P>, PriorityQueue>(initial_state, invalid_action, 0);
}

template <class P, class A, class HF>
std::vector<A> iterative_deepening_a_star(const P &initial_state, const A &invalid_action) {
    return deepening_graph_search<P, A, HF, PriorityQueue>(initial_state, invalid_action, 0);
}

template <class P, class A, class HF, template <class, class> class Q>
std::vector<A> tree_search(const P &initial_state, const A &invalid_action) {
    return deepening_tree_search<P, A, HF, Q>(initial_state, invalid_action,
                                              BaseSearchIterator<P, A, HF, Q>::MAX_COST);
}

template <class P, class A, class HF, template <class, class> class Q>
std::vector<A> graph_search(const P &initial_state, const A &invalid_action) {
    return deepening_graph_search<P, A, HF, Q>(initial_state, invalid_action,
                                               BaseSearchIterator<P, A, HF, Q>::MAX_COST);
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

template <class P, class A>
std::vector<A> bidirectional_bfs(const P &initial_state, const A &invalid_action) {
    return bidirectional_search<P, A, NoHeuristic<P>, Queue, GraphSearchIterator>(initial_state, invalid_action);
}

#endif
