#pragma once

namespace libMultiRobotPlanning
{
    /*! \brief Represents state transations

        This class represents a transition from a start state applying an action
       with the given cost.

        \tparam State Custom state for the search. Needs to be copy'able
        \tparam Action Custom action for the search. Needs to be copy'able
        \tparam Cost Custom Cost type (integer or floating point types)
    */
    template <typename State, typename Action, typename Cost>
    struct Neighbor
    {
        //! neighboring state
        State state;
        //! action to get to the neighboring state
        Action action;
        //! cost to get to the neighboring state
        Cost cost;

        Neighbor(const State& input_state, const Action& input_action, Cost input_cost)
          : state(input_state),
          action(input_action),
          cost(input_cost)
          {}
    };

}  // namespace libMultiRobotPlanning
