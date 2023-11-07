//
// Created by take_ on 2023/11/6.
//

#ifndef LIBMULTIROBOTPLANNING_A_STAR_ISOLATED_HPP
#define LIBMULTIROBOTPLANNING_A_STAR_ISOLATED_HPP

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "neighbor.hpp"
#include "planresult.hpp"
#include "util.hpp"

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

enum class Action // 模板类Action实例化
{
    Up,
    Down,
    Left,
    Right,
};

class AStarNode
{
public:
    Location location;
    int f_score;
    int g_score;

    // 定义 handle: 就是上面那个HeapHandle
    typename boost::heap::fibonacci_heap<AStarNode>::handle_type handle;
    // typename boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

public:
    AStarNode(const Location& input_state, int input_fScore, int input_gScore)
            : location(input_state),
              f_score(input_fScore),
              g_score(input_gScore)
    {}

    bool operator<(const AStarNode& other) const
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

    friend std::ostream& operator<<(std::ostream& os, const AStarNode& node)
    {
        os << "location: " << node.location << " f_score: " << node.f_score
           << " g_score: " << node.g_score;

        return os;
    }
};

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

  - `void get_neighbors(const Location& s, std::vector<Neighbor<Location, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring location for the given location s.

*/

class Environment // 模板类Environment实例化
{
private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location start;
    Location goal;

public:
    Environment(size_t input_num_columns, size_t input_num_rows,
                std::unordered_set<Location> input_obstacles, Location input_start, Location input_goal)
            : num_columns(input_num_columns),
              num_rows(input_num_rows),
              obstacles(std::move(input_obstacles)),
              start(std::move(input_start)),
              goal(std::move(input_goal))
    {}

    // whether location valid or not
    bool location_valid(const Location& location)
    {
        return location.x >= 0 && location.x < num_columns
               && location.y >= 0 && location.y < num_rows
               && obstacles.find(location) == obstacles.end();
    }

    // get neighbor of current location
    void get_neighbors(const Location& location, std::vector<Neighbor<Location, Action, int> >& neighbors)
    {
        neighbors.clear();

        Location north_neighbor(location.x, location.y + 1);

        if (location_valid(north_neighbor))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(north_neighbor, Action::Up, 1));
        }

        Location south_neighbor(location.x, location.y - 1);

        if (location_valid(south_neighbor))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(south_neighbor, Action::Down, 1));
        }

        Location west_neighbor(location.x - 1, location.y);

        if (location_valid(west_neighbor))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(west_neighbor, Action::Left, 1));
        }

        Location east_neighbor(location.x + 1, location.y);

        if (location_valid(east_neighbor))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(east_neighbor, Action::Right, 1));
        }
    }
};

class AStar
{
private:
    // member vars
    Environment environment; // include map size, obstacle position, agent goal.
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location start;
    Location goal;

    // 定义openSet_t和fibHeapHandle_t
    using OpenSet = boost::heap::fibonacci_heap<AStarNode>;
    using HeapHandle = typename OpenSet::handle_type;
    // using OpenSet = boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
    // using HeapHandle = typename OpenSet::handle_type;

public:
    // member funcs
    AStar(Environment input_environment, size_t input_num_columns, size_t input_num_rows,
          std::unordered_set<Location> input_obstacles, Location input_start, Location input_goal)
          : environment(input_environment),
            num_columns(input_num_columns),
            num_rows(input_num_rows),
            obstacles(std::move(input_obstacles)),
            start(std::move(input_start)),
            goal(std::move(input_goal))
    {}

    int admissible_heuristic(const Location& current_location)
    {
        return abs(current_location.x - goal.x)
               + abs(current_location.y - goal.y);
    }

    // whether agent reach goal or not.
    bool is_solution(const Location& current_location)
    {
        return current_location == goal;
    }

    // This function is called on every expansion and can be used for statistical purposes.
    void onExpandNode(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}

    // This function is called on every node discovery and can be used for statistical purposes.
    void onDiscover(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}

    bool a_star_search(const Location& start_location, PlanResult<Location, Action, int>& solution,
                       int initialCost = 0)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start_location, 0));
        solution.actions.clear();
        solution.cost = 0;

        OpenSet open_set;
        std::unordered_map<Location, HeapHandle, std::hash<Location>> location_to_heap;
        std::unordered_set<Location, std::hash<Location>> closed_set;
        std::unordered_map<Location, std::tuple<Location,Action,int,int>,std::hash<Location>> came_from;

        auto handle = open_set.push(AStarNode(start_location,
  admissible_heuristic(start_location), initialCost));
        location_to_heap.insert(std::make_pair<>(start_location, handle));
        (*handle).handle = handle;

        std::vector<Neighbor<Location, Action, int> > neighbors;
        neighbors.reserve(10);

        while (!open_set.empty())
        {
            AStarNode current = open_set.top();
            onExpandNode(current.location, current.f_score, current.g_score);

            if (is_solution(current.location))
            {
                solution.path.clear();
                solution.actions.clear();
                auto iter = came_from.find(current.location);
                while (iter != came_from.end())
                {
                    solution.path.emplace_back(
                            std::make_pair<>(iter->first, std::get<3>(iter->second)));
                    solution.actions.emplace_back(std::make_pair<>(
                            std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = came_from.find(std::get<0>(iter->second));
                }

                solution.path.emplace_back(std::make_pair<>
                                                   (start_location, initialCost));
                std::reverse(solution.path.begin(), solution.path.end());
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
            for (const Neighbor<Location, Action, int>& neighbor : neighbors)
            {
                if (closed_set.find(neighbor.location) == closed_set.end())
                {
                    int tentative_gScore = current.g_score + neighbor.cost;
                    auto iter = location_to_heap.find(neighbor.location);
                    if (iter == location_to_heap.end())
                    {  // Discover a new node
                        int f_score = tentative_gScore + admissible_heuristic(neighbor.location);
                        auto handle = open_set.push(AStarNode(neighbor.location, f_score, tentative_gScore));
                        (*handle).handle = handle;
                        location_to_heap.insert(std::make_pair<>(neighbor.location, handle));
                        onDiscover(neighbor.location, f_score, tentative_gScore);
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
                        int delta = (*handle).g_score - tentative_gScore;
                        (*handle).g_score = tentative_gScore;
                        (*handle).f_score -= delta;
                        open_set.increase(handle);
                        onDiscover(neighbor.location, (*handle).f_score, (*handle).g_score);
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


#endif //LIBMULTIROBOTPLANNING_A_STAR_ISOLATED_HPP
