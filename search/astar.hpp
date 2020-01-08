#ifndef DENIZMSAYIN_SBLOCK_SEARCH_ASTAR_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_ASTAR_HPP

// Could not observe a speed difference between previous version
// and simply using WA* with -O3, which is why A* calls WA*

namespace denizmsayin::sblock::search {

    template <class Puzzle, class Action, class HeuristicFunc, 
              typename Cost = default_cost_t, bool Rev = false>
    SearchResult a_star_search(const Puzzle &p, HeuristicFunc hf=HeuristicFunc()) {
        return weighted_a_star_search<Puzzle, Action, HeuristicFunc, Cost, Rev>(p, 1.0, hf);
    }

}
#endif
