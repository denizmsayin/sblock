#ifndef DENIZMSAYIN_SBLOCK_SEARCH_ACTION_EXT_HPP
#define DENIZMSAYIN_SBLOCK_SEARCH_ACTION_EXT_HPP
#include <optional>

namespace denizmsayin::sblock::search {
    // A template class that allows extending a node type with action information
    // Contains lots of forwarding for easy construction of base types.
    // Kind of like the decorator pattern, but a bit ugly due to being static.
    // Optional, because the first move would not contain any previous actions.
    template <typename Action, typename BaseNode> 
    struct ActionExtended : public BaseNode {
        std::optional<Action> action;

        // variadic template constructor forwarded to base class
        template <typename... Args>
        ActionExtended(const Action &a, Args&&... args) 
            : BaseNode(std::forward<Args>(args)...), action(a) {}
        
        template <typename... Args>
        ActionExtended(std::nullopt_t, Args&&... args) 
            : BaseNode(std::forward<Args>(args)...), action() {}
   
        template <typename... Args>
        ActionExtended(const std::optional<Action> &a, Args&&... args) 
            : BaseNode(std::forward<Args>(args)...), action(a) {}

    };

    // Easy interface for action extended construction
    // for optional speed-ups in different search functions
    // quite critical for IDA*. These functions do different things
    // depending on their boolean template parameter, but these are
    // determined at compile time, which means almost no performance 
    // hit is incurred.

    template <bool Extend, class Action, class BaseNode>
    using MaybeActionExtended = typename std::conditional<Extend,
                                                          ActionExtended<Action, BaseNode>,
                                                          BaseNode>::type;

    // construct either a normal node, or a node with an action depending on Extend
    template <bool Extend, class Action, class BaseNode, typename... Args>
    typename MaybeActionExtended<Extend, Action, BaseNode>
    maybe_construct_action_extended_node(
            const std::optional<Action> &action, 
            Args&&... args) 
    {
        if constexpr (Extend) {
            return ActionExtended<Action, BaseNode>(action, std::forward<Args>(args)...);
        } else {
            return BaseNode(std::forward<Args>(args)...);
        }
    }

    // emplace either a normal node, or a node with an action depending on Extend
    template <bool Extend, class Action, class QueueType, typename... Args>
    void maybe_emplace_action_extended_node(
            QueueType &q, // std::stack, queue, priority_queue... 
            const std::optional<Action> &action, 
            Args&&... args) 
    {
        if constexpr (Extend) {
            q.emplace(action, std::forward<Args>(args)...);
        } else {
            q.emplace(std::forward<Args>(args)...);
        }
    }

    // either reverse the input action, or return an empty optional depending on Reverse
    template <bool Reverse, class Action, class Node>
    std::optional<Action> maybe_reverse_action(const Node &n) {
        if constexpr (Reverse) {
            if(n.action.has_value())
                return Action::reverse(n.action.value());
            else
                return std::nullopt;
        } else {
            return std::nullopt;
        }
    }

    // either retrieve an action from the node, or return empty optional depending on Reverse
    template <bool Reverse, class Action, class Node>
    std::optional<Action> maybe_retrieve_action(const Node &n) {
        if constexpr (Reverse) {
            return n.action;
        } else {
            return std::nullopt;
        }
    }
}
#endif
