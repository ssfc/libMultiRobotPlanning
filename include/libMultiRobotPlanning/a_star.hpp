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

\tparam Location Custom location for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom A* logic. In
    particular, it needs to support the following functions:
  - `Cost admissible_heuristic(const Location& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `bool is_solution(const Location& s)`\n
    Return true if the given location is a goal location.

  - `void get_neighbors(const Location& s, std::vector<Neighbor<Location, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring location for the given location s.

  - `void onExpandNode(const Location& s, int f_score, int g_score)`\n
    This function is called on every expansion and can be used for statistical
purposes.

  - `void onDiscover(const Location& s, int f_score, int g_score)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

    \tparam StateHasher A class to convert a location to a hash value. Default:
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
            Location location;
            Cost f_score;
            Cost g_score;

            // 定义 handle
            typename boost::heap::fibonacci_heap<Node>::handle_type handle;
            // typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

        public:
            Node(const Location& input_state, Cost input_fScore, Cost input_gScore)
                    : location(input_state),
                    f_score(input_fScore),
                    g_score(input_gScore)
                    {}

            bool operator<(const Node& other) const
            {
                // Sort order
                // 1. lowest f_score
                // 2. highest g_score

                // Our heap is a maximum heap, so we invert the comperator function here
                if (f_score != other.f_score)
                {
                    return f_score > other.f_score;
                }
                else
                {
                    return g_score < other.g_score;
                }
            }

            friend std::ostream& operator<<(std::ostream& os, const Node& node)
            {
                os << "location: " << node.location << " f_score: " << node.f_score
                   << " g_score: " << node.g_score;

                return os;
            }
        };

        // 定义openSet_t和fibHeapHandle_t
        using openSet_t = boost::heap::fibonacci_heap<Node>;
        using fibHeapHandle_t = typename openSet_t::handle_type;
        // using openSet_t = boost::heap::d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true>>;
        // using fibHeapHandle_t = typename openSet_t::handle_type;

        Environment& environment;

    public:
        AStar(Environment& input_environment) : environment(input_environment) {}

        bool search(const Location& startState,
                  PlanResult<Location, Action, Cost>& solution, Cost initialCost = 0)
        {
            solution.locations.clear();
            solution.locations.push_back(std::make_pair<>(startState, 0));
            solution.actions.clear();
            solution.cost = 0;

            openSet_t open_set;
            std::unordered_map<Location, fibHeapHandle_t, StateHasher> location_to_heap;
            std::unordered_set<Location, StateHasher> closed_set;
            std::unordered_map<Location, std::tuple<Location,Action,Cost,Cost>,StateHasher> came_from;

            auto handle = open_set.push(Node(startState, environment.admissible_heuristic(startState), initialCost));
            location_to_heap.insert(std::make_pair<>(startState, handle));
            (*handle).handle = handle;

            std::vector<Neighbor<Location, Action, Cost> > neighbors;
            neighbors.reserve(10);

            while (!open_set.empty())
            {
                Node current = open_set.top();

                if (environment.is_solution(current.location))
                {
                    solution.locations.clear();
                    solution.actions.clear();
                    auto iter = came_from.find(current.location);
                    while (iter != came_from.end())
                    {
                        solution.locations.push_back(
                          std::make_pair<>(iter->first, std::get<3>(iter->second)));
                        solution.actions.push_back(std::make_pair<>(
                          std::get<1>(iter->second), std::get<2>(iter->second)));
                        iter = came_from.find(std::get<0>(iter->second));
                    }

                    solution.locations.push_back(std::make_pair<>(startState, initialCost));
                    std::reverse(solution.locations.begin(), solution.locations.end());
                    std::reverse(solution.actions.begin(), solution.actions.end());
                    solution.cost = current.g_score;
                    solution.fmin = current.f_score;

                    return true;
                }

                open_set.pop();
                location_to_heap.erase(current.location);
                closed_set.insert(current.location);

                // traverse neighbors
                neighbors.clear();
                environment.get_neighbors(current.location, neighbors);
                for (const Neighbor<Location, Action, Cost>& neighbor : neighbors)
                {
                    if (closed_set.find(neighbor.location) == closed_set.end())
                    {
                        Cost tentative_gScore = current.g_score + neighbor.cost;
                        auto iter = location_to_heap.find(neighbor.location);
                        if (iter == location_to_heap.end())
                        {  // Discover a new node
                            Cost f_score = tentative_gScore + environment.admissible_heuristic(neighbor.location);
                            auto handle = open_set.push(Node(neighbor.location, f_score, tentative_gScore));
                            (*handle).handle = handle;
                            location_to_heap.insert(std::make_pair<>(neighbor.location, handle));
                            // std::cout << "  this is a new node " << f_score << "," <<
                            // tentative_gScore << std::endl;
                        }
                        else
                        {
                            auto handle = iter->second;
                            // std::cout << "  this is an old node: " << tentative_gScore << ","
                            // << (*handle).g_score << std::endl;
                            // We found this node before with a better path
                            if (tentative_gScore >= (*handle).g_score)
                            {
                                continue;
                            }

                            // update f and g_score
                            Cost delta = (*handle).g_score - tentative_gScore;
                            (*handle).g_score = tentative_gScore;
                            (*handle).f_score -= delta;
                            open_set.increase(handle);
                        }

                        // Best path for this node so far
                        // TODO: this is not the best way to update "came_from", but otherwise
                        // default c'tors of Location and Action are required
                        came_from.erase(neighbor.location);
                        came_from.insert(std::make_pair<>(neighbor.location,
                          std::make_tuple<>(current.location, neighbor.action, neighbor.cost,
                                            tentative_gScore)));
                    }
                }
            }

            return false;
        }
    };

}  // namespace libMultiRobotPlanning
