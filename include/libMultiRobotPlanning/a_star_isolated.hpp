//
// Created by take_ on 2023/11/6.
//

#ifndef A_STAR_ISOLATED_HPP
#define A_STAR_ISOLATED_HPP

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "util.hpp"


enum class Action // 模板类Action实例化
{
    Up,
    Down,
    Left,
    Right,
};

struct Child
{
    //! neighboring location
    Location location;
    //! action to get to the neighboring location
    Action action;
    //! cost to get to the neighboring location, usually 1
    int cost;

    Child(const Location& input_location, const Action& input_action, int input_cost)
            : location(input_location),
              action(input_action),
              cost(input_cost)
    {}
};

struct AgentPlan
{
    // path constructing locations and their g_score
    std::vector<std::pair<Location, int> > path;
    //! actions and their cost
    std::vector<std::pair<Action, int> > actions;
    //! actual cost of the result
    int cost;
    //! lower bound of the cost (for suboptimal solvers)
    int fmin;
};

class AStarNode
{
public:
    Location location;
    int f_score;
    int g_score;

    // 定义 handle: 就是上面那个HeapHandle
    // typename boost::heap::fibonacci_heap<AStarNode>::handle_type handle;
    typename boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

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
*/

class AStar
{
private:
    // member vars
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location start;
    Location goal;

    // debug var;
    size_t num_expanded_nodes;
    size_t num_generated_nodes;

    // 定义openSet_t和fibHeapHandle_t
    // using OpenSet = boost::heap::fibonacci_heap<AStarNode>;
    // using HeapHandle = typename OpenSet::handle_type;
    using HeapHandle = typename boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>>
        ::handle_type;

public:
    // member funcs
    AStar(size_t input_num_columns, size_t input_num_rows,
          std::unordered_set<Location> input_obstacles, Location input_start, Location input_goal)
          : num_columns(input_num_columns),
            num_rows(input_num_rows),
            obstacles(std::move(input_obstacles)),
            start(std::move(input_start)),
            goal(std::move(input_goal)),
            num_expanded_nodes(0),
            num_generated_nodes(0)
    {}

    // This function can return 0 if no suitable heuristic is available.
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

    // whether location valid or not
    bool location_valid(const Location& location)
    {
        return location.x >= 0 && location.x < num_columns
               && location.y >= 0 && location.y < num_rows
               && obstacles.find(location) == obstacles.end();
    }

    // get neighbor of current location
    void get_neighbors(const Location& location, std::vector<Child>& children)
    {
        children.clear();

        Location north_neighbor(location.x, location.y + 1);

        if (location_valid(north_neighbor))
        {
            children.emplace_back(Child(north_neighbor, Action::Up, 1));
        }

        Location south_neighbor(location.x, location.y - 1);

        if (location_valid(south_neighbor))
        {
            children.emplace_back(Child(south_neighbor, Action::Down, 1));
        }

        Location west_neighbor(location.x - 1, location.y);

        if (location_valid(west_neighbor))
        {
            children.emplace_back(Child(west_neighbor, Action::Left, 1));
        }

        Location east_neighbor(location.x + 1, location.y);

        if (location_valid(east_neighbor))
        {
            children.emplace_back(Child(east_neighbor, Action::Right, 1));
        }
    }


    bool a_star_search(AgentPlan& solution, int initialCost = 0)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start, 0));
        solution.actions.clear();
        solution.cost = 0;

        boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>> open_set;
        std::unordered_map<Location, HeapHandle, std::hash<Location>> location_to_heap;
        std::unordered_set<Location, std::hash<Location>> closed_set;
        std::unordered_map<Location, std::tuple<Location,Action,int,int>,std::hash<Location>> came_from;

        auto current_node_handle = open_set.emplace(AStarNode(start, admissible_heuristic(start), initialCost));
        location_to_heap.insert(std::make_pair<>(start, current_node_handle));
        (*current_node_handle).handle = current_node_handle;

        std::vector<Child> children;
        children.reserve(10);

        while (!open_set.empty())
        {
            AStarNode current = open_set.top();
            num_expanded_nodes++;

            if (is_solution(current.location))
            {
                solution.path.clear();
                solution.actions.clear();
                // std::unordered_map<Location, std::tuple<Location,Action,int,int>,std::hash<Location>> came_from;
                auto iter = came_from.find(current.location);
                while (iter != came_from.end())
                {
                    // iter->first是location
                    // iter->second是std::tuple<Location,Action,int,int>
                    solution.path.emplace_back(
                            std::make_pair<>(iter->first, std::get<3>(iter->second)));
                    solution.actions.emplace_back(std::make_pair<>(
                            std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = came_from.find(std::get<0>(iter->second));
                }

                solution.path.emplace_back(std::make_pair<>(start, initialCost));
                std::reverse(solution.path.begin(), solution.path.end());
                std::reverse(solution.actions.begin(), solution.actions.end());
                solution.cost = current.g_score;
                solution.fmin = current.f_score;

                std::cerr << "num expanded nodes: " << num_expanded_nodes << std::endl;
                std::cerr << "num generated nodes: " << num_generated_nodes << std::endl;

                return true;
            }

            open_set.pop();
            location_to_heap.erase(current.location);
            closed_set.insert(current.location);

            // traverse children
            children.clear();
            get_neighbors(current.location, children);
            for (const Child& neighbor : children)
            {
                if (closed_set.find(neighbor.location) == closed_set.end()) // not in closed set
                {
                    int tentative_gScore = current.g_score + neighbor.cost;
                    auto iter = location_to_heap.find(neighbor.location);
                    if (iter == location_to_heap.end())
                    {  // Discover a new node
                        int f_score = tentative_gScore + admissible_heuristic(neighbor.location);
                        auto handle = open_set.emplace(AStarNode(neighbor.location, f_score, tentative_gScore));
                        (*handle).handle = handle;
                        location_to_heap.insert(std::make_pair<>(neighbor.location, handle));
                        num_generated_nodes++;
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
                        num_generated_nodes++;
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


#endif //A_STAR_ISOLATED_HPP
