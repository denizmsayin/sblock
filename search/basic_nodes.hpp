#ifndef DENIZMSAYIN_SBLOCK_SEARCH_BASIC_NODES_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_BASIC_NODES_HPP

namespace denizmsayin::sblock::search {
    // SEARCH NODE, the class that defines a search node, a record structure that keeps
    // both a puzzle state and extra book-keeping information about it.
    // It is a templated class because what is contained can change depending on the algorithm.
    // i.e. a const Puzzle * for BFS, or a pair<Puzzle, int> * for A* search etc.
    
    template <typename Contained, typename Cost = default_cost_t> 
    struct PNode {
        Contained entry;
        Cost path_cost;

        PNode(const Contained &p, Cost pc) 
            : entry(p), path_cost(pc) {}
    };


    template <typename Contained, typename Cost = default_cost_t> 
    struct HNode : PNode<Contained, Cost> {
        Cost est_cost;

        HNode(const Contained &p, Cost pc, Cost ec) 
            : PNode<Contained, Cost>(p, pc), est_cost(ec) {}

        bool operator<(const HNode &other) const { return est_cost < other.est_cost; }
        bool operator>(const HNode &other) const { return est_cost > other.est_cost; }
    };
}
#endif
