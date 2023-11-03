#pragma once

namespace libMultiRobotPlanning
{
    /*! \brief Represents state transations

        This class represents a transition from a start state applying an action
       with the given cost.

        \tparam Location Custom state for the search. Needs to be copy'able
        \tparam Action Custom action for the search. Needs to be copy'able
        \tparam Cost Custom Cost type (integer or floating point types)
    */
    template <typename Location, typename Action, typename Cost>
    struct Neighbor
    {
        //! neighboring state
        Location state;
        //! action to get to the neighboring state
        Action action;
        //! cost to get to the neighboring state
        Cost cost;

        Neighbor(const Location& input_location, const Action& input_action, Cost input_cost)
          : state(input_location),
          action(input_action),
          cost(input_cost)
          {}
    };

}  // namespace libMultiRobotPlanning
