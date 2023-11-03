#pragma once

namespace libMultiRobotPlanning
{
    /*! \brief Represents location transations

        This class represents a transition from a start location applying an action
       with the given cost.

        \tparam Location Custom location for the search. Needs to be copy'able
        \tparam Action Custom action for the search. Needs to be copy'able
        \tparam Cost Custom Cost type (integer or floating point types)
    */
    template <typename Location, typename Action, typename Cost>
    struct Neighbor
    {
        //! neighboring location
        Location location;
        //! action to get to the neighboring location
        Action action;
        //! cost to get to the neighboring location
        Cost cost;

        Neighbor(const Location& input_location, const Action& input_action, Cost input_cost)
          : location(input_location),
          action(input_action),
          cost(input_cost)
          {}
    };

}  // namespace libMultiRobotPlanning
