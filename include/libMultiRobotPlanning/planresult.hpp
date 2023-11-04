#pragma once

#include <vector>

namespace libMultiRobotPlanning
{

/*! \brief Represents the path for an agent

    This class is used to store the result of a planner for a single agent.
    It has both the ordered list of locations that need to be traversed as well as
   the ordered
    list of actions together with their respective costs

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
    template <typename State, typename Action, typename Cost>
    struct PlanResult
    {
        // path constructing locations and their g_score
        std::vector<std::pair<State, Cost> > locations;
        //! actions and their cost
        std::vector<std::pair<Action, Cost> > actions;
        //! actual cost of the result
        Cost cost;
        //! lower bound of the cost (for suboptimal solvers)
        Cost fmin;
    };

}  // namespace libMultiRobotPlanning
