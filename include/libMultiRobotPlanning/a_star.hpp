#pragma once

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"

namespace libMultiRobotPlanning
{

/*!
  \example a_star.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief A* Algorithm to find the shortest path

This class implements the A* algorithm. A* is an informed search algorithm
that finds the shortest path for a given map. It can use a heuristic that
needsto be admissible.

This class can either use a fibonacci heap, or a d-ary heap. The latter is the
default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap instead.

\tparam Location Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom A* logic. In
    particular, it needs to support the following functions:
  - `Cost admissible_heuristic(const Location& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `bool is_solution(const Location& s)`\n
    Return true if the given state is a goal state.

  - `void get_neighbors(const Location& s, std::vector<Neighbor<Location, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring state for the given state s.

  - `void onExpandNode(const Location& s, int fScore, int gScore)`\n
    This function is called on every expansion and can be used for statistical
purposes.

  - `void onDiscover(const Location& s, int fScore, int gScore)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

    \tparam StateHasher A class to convert a state to a hash value. Default:
   std::hash<Location>
*/
    template <typename Location, typename Action, typename Cost, typename Environment,
              typename StateHasher = std::hash<Location> >
    class AStar
    {
    private:
        class Node
        {
        public:
            Location state;
            Cost fScore;
            Cost gScore;

            // 定义 handle
            typename boost::heap::fibonacci_heap<Node>::handle_type handle;
            // typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

        public:
            Node(const Location& input_state, Cost input_fScore, Cost input_gScore)
                    : state(input_state),
                    fScore(input_fScore),
                    gScore(input_gScore)
                    {}

            bool operator<(const Node& other) const
            {
                // Sort order
                // 1. lowest fScore
                // 2. highest gScore

                // Our heap is a maximum heap, so we invert the comperator function here
                if (fScore != other.fScore)
                {
                    return fScore > other.fScore;
                }
                else
                {
                    return gScore < other.gScore;
                }
            }

            friend std::ostream& operator<<(std::ostream& os, const Node& node)
            {
                os << "state: " << node.state << " fScore: " << node.fScore
                   << " gScore: " << node.gScore;

                return os;
            }
        };

        // 定义openSet_t和fibHeapHandle_t
        typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
        typedef typename openSet_t::handle_type fibHeapHandle_t;
        // typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true>> openSet_t;
        // typedef typename openSet_t::handle_type fibHeapHandle_t;

        Environment& environment;

    public:
        AStar(Environment& input_environment) : environment(input_environment) {}

        bool search(const Location& startState,
                  PlanResult<Location, Action, Cost>& solution, Cost initialCost = 0)
        {
            solution.states.clear();
            solution.states.push_back(std::make_pair<>(startState, 0));
            solution.actions.clear();
            solution.cost = 0;

            openSet_t open_set;
            std::unordered_map<Location, fibHeapHandle_t, StateHasher> stateToHeap;
            std::unordered_set<Location, StateHasher> closedSet;
            std::unordered_map<Location, std::tuple<Location,Action,Cost,Cost>,StateHasher> cameFrom;

            auto handle = open_set.push(Node(startState, environment.admissible_heuristic(startState), initialCost));
            stateToHeap.insert(std::make_pair<>(startState, handle));
            (*handle).handle = handle;

            std::vector<Neighbor<Location, Action, Cost> > neighbors;
            neighbors.reserve(10);

            while (!open_set.empty())
            {
                Node current = open_set.top();

                if (environment.is_solution(current.state))
                {
                    solution.states.clear();
                    solution.actions.clear();
                    auto iter = cameFrom.find(current.state);
                    while (iter != cameFrom.end())
                    {
                        solution.states.push_back(
                          std::make_pair<>(iter->first, std::get<3>(iter->second)));
                        solution.actions.push_back(std::make_pair<>(
                          std::get<1>(iter->second), std::get<2>(iter->second)));
                        iter = cameFrom.find(std::get<0>(iter->second));
                    }

                    solution.states.push_back(std::make_pair<>(startState, initialCost));
                    std::reverse(solution.states.begin(), solution.states.end());
                    std::reverse(solution.actions.begin(), solution.actions.end());
                    solution.cost = current.gScore;
                    solution.fmin = current.fScore;

                    return true;
                }

                open_set.pop();
                stateToHeap.erase(current.state);
                closedSet.insert(current.state);

                // traverse neighbors
                neighbors.clear();
                environment.get_neighbors(current.state, neighbors);
                for (const Neighbor<Location, Action, Cost>& neighbor : neighbors)
                {
                    if (closedSet.find(neighbor.state) == closedSet.end())
                    {
                        Cost tentative_gScore = current.gScore + neighbor.cost;
                        auto iter = stateToHeap.find(neighbor.state);
                        if (iter == stateToHeap.end())
                        {  // Discover a new node
                            Cost fScore = tentative_gScore + environment.admissible_heuristic(neighbor.state);
                            auto handle = open_set.push(Node(neighbor.state, fScore, tentative_gScore));
                            (*handle).handle = handle;
                            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
                            // std::cout << "  this is a new node " << fScore << "," <<
                            // tentative_gScore << std::endl;
                        }
                        else
                        {
                            auto handle = iter->second;
                            // std::cout << "  this is an old node: " << tentative_gScore << ","
                            // << (*handle).gScore << std::endl;
                            // We found this node before with a better path
                            if (tentative_gScore >= (*handle).gScore)
                            {
                                continue;
                            }

                            // update f and gScore
                            Cost delta = (*handle).gScore - tentative_gScore;
                            (*handle).gScore = tentative_gScore;
                            (*handle).fScore -= delta;
                            open_set.increase(handle);
                        }

                        // Best path for this node so far
                        // TODO: this is not the best way to update "cameFrom", but otherwise
                        // default c'tors of Location and Action are required
                        cameFrom.erase(neighbor.state);
                        cameFrom.insert(std::make_pair<>(neighbor.state,
                          std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                            tentative_gScore)));
                    }
                }
            }

            return false;
        }
    };

}  // namespace libMultiRobotPlanning
