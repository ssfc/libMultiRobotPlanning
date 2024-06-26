//
// Created by take_ on 2023/11/6.
//

#ifndef A_STAR_ISOLATED_HPP
#define A_STAR_ISOLATED_HPP

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include "include/libMultiRobotPlanning/util.hpp"


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

class AStar
{
private:
    // member vars
    size_t num_columns;
    size_t num_rows;
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
            start(input_start),
            goal(input_goal),
            num_expanded_nodes(0),
            num_generated_nodes(0)
    {}

    // This function can return 0 if no suitable heuristic is available.
    int calculate_h(const Location& current_location) const
    {
        return abs(current_location.x - goal.x) + abs(current_location.y - goal.y);
    }

    // whether agent reach goal or not.
    bool is_goal(const Location& current_location)
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
    std::vector<Child> get_neighbors(const Location& location)
    {
        std::vector<Child> children;

        Location north_neighbor(location.x, location.y + 1);

        if (location_valid(north_neighbor))
        {
            children.emplace_back(north_neighbor, Action::Up, 1);
        }

        Location south_neighbor(location.x, location.y - 1);

        if (location_valid(south_neighbor))
        {
            children.emplace_back(south_neighbor, Action::Down, 1);
        }

        Location west_neighbor(location.x - 1, location.y);

        if (location_valid(west_neighbor))
        {
            children.emplace_back(west_neighbor, Action::Left, 1);
        }

        Location east_neighbor(location.x + 1, location.y);

        if (location_valid(east_neighbor))
        {
            children.emplace_back(east_neighbor, Action::Right, 1);
        }

        return children;
    }

    // A* LINE 1
    // A* finds a path from start to goal.
    // h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    // function A_Star(start, goal, h_score)
    bool a_star_search(AgentPlan& solution)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start, 0));
        solution.actions.clear();
        solution.cost = 0;

        boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>> open_set;
        std::unordered_map<Location, HeapHandle, std::hash<Location>> location_to_heaphandle;

        // A* LINE 2
        // For node n, came_from[n] is the node immediately preceding it on the cheapest path from the start
        // to n currently known.
        // came_from := an empty map
        std::unordered_map<Location, std::tuple<Location,Action,int,int>,std::hash<Location>> came_from;

        // A* LINE 3
        // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
        // gScore := map with default value of Infinity

        // A* LINE 4
        // gScore[start] := 0

        // A* LINE 5
        // For node n, fScore[n] := gScore[n] + h_score(n). fScore[n] represents our current best guess as to
        // how cheap a path could be from start to finish if it goes through n.
        // fScore := map with default value of Infinity

        // A* LINE 6
        // fScore[start] := h_score(start)
        // Initialising the parameters of the starting node

        // A* LINE 7
        // The set of discovered nodes that may need to be (re-)expanded.
        // Initially, only the start node is known.
        // This is usually implemented as a min-heap or priority queue rather than a hash-set.
        // open_set := {start}
        auto root_handle = open_set.emplace(AStarNode(start, calculate_h(start), 0));
        location_to_heaphandle.insert(std::make_pair<>(start, root_handle));

        // A* LINE 8
        // node that has already been evaluated. In other words, already been poped from open_set.
        // closedset := the empty set
        std::unordered_set<Location, std::hash<Location>> closed_set;

        // A* LINE 9
        // while openSet is not empty
        while (!open_set.empty())
        {
            // A* LINE 10
            // This operation can occur in O(Log(N)) time if open_set is a min-heap or a priority queue
            // current := the node in open_set having the lowest f_score[] value
            AStarNode current = open_set.top();
            num_expanded_nodes++;

            // A* LINE 11
            // if current = goal
            if (is_goal(current.location))
            {
                solution.path.clear();
                solution.actions.clear();

                // A* LINE 12
                // total_path := {current}
                solution.path.emplace(solution.path.begin(), std::make_pair<>(current.location, came_from.size()));

                // A* LINE 13
                // while current in came_from.Keys:
                auto iter = came_from.find(current.location);
                while (iter != came_from.end())
                {
                    // A* LINE 14
                    // current := came_from[current]

                    // A* LINE 15
                    // total_path.prepend(current)

                    // iter->first是location
                    // iter->second是std::tuple<Location,Action,int,int>
                    solution.path.emplace(solution.path.begin(),
                        std::make_pair<>(std::get<0>(iter->second), std::get<3>(iter->second)));
                    solution.actions.emplace(solution.actions.begin(),
                        std::make_pair<>(std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = came_from.find(std::get<0>(iter->second)); // find(Location)
                }

                // A* LINE 16
                // return total_path

                solution.cost = current.g_score;

                std::cerr << "num expanded nodes: " << num_expanded_nodes << std::endl;
                std::cerr << "num generated nodes: " << num_generated_nodes << std::endl;

                return true;
            }

            // A* LINE 17
            // openSet.Remove(current)
            open_set.pop();
            location_to_heaphandle.erase(current.location);
            // A* LINE 18
            // add current to closedset.
            closed_set.insert(current.location);

            // traverse children
            auto children = get_neighbors(current.location);
            // A* LINE 19
            // for each neighbor of current
            for (const Child& neighbor : children)
            {
                // A* LINE 20
                // if neighbor not in closedset
                if (closed_set.find(neighbor.location) == closed_set.end()) // not in closed set
                {
                    // A* LINE 21
                    // d(current,neighbor) is the weight of the edge from current to neighbor
                    // tentative_g_score is the distance from start to the neighbor through current
                    // tentative_g_score := gScore[current] + d(current, neighbor)
                    // cost of the cheapest path from start to n currently known
                    int tentative_g_score = current.g_score + neighbor.cost;
                    auto map_found = location_to_heaphandle.find(neighbor.location);

                    // A* LINE 22
                    // 坐标不在堆中就不比较直接加（毕竟原g值是inf, 新g值肯定更小）
                    if (map_found == location_to_heaphandle.end())
                    {
                        // A* LINE 23
                        // cameFrom[neighbor] := current
                        came_from.insert(std::make_pair<>(neighbor.location,
                          std::make_tuple<>(current.location, neighbor.action, neighbor.cost, tentative_g_score)));

                        // A* LINE 24
                        // g_score[neighbor] := tentative_gScore
                        // A* LINE 25
                        // f_score[neighbor] := tentative_gScore + h(neighbor)
                        // A* LINE 26
                        // open_set.add(neighbor)
                        int f_score = tentative_g_score + calculate_h(neighbor.location);
                        auto neighbor_handle = open_set.emplace(AStarNode(neighbor.location, f_score, tentative_g_score));
                        location_to_heaphandle.insert(std::make_pair<>(neighbor.location, neighbor_handle));
                        num_generated_nodes++;
                        // std::cout << "  this is a new node " << f_score << "," <<
                        // tentative_g_score << std::endl;
                    }
                    // A* LINE 27
                    else // 坐标在堆中。
                    {
                        auto neighbor_handle =
                            map_found->second; // Location所具有的其他特征。
                        // std::cout << "  this is an old node: " << tentative_g_score << ","
                        // << (*handle).g_score << std::endl;

                        // A* LINE 28
                        // if tentative_gScore < gScore[neighbor]
                        // meaning: This path to neighbor is better than any previous one. Record it!
                        // We found this node before with a better path
                        if (tentative_g_score < (*neighbor_handle).g_score)
                        {
                            // A* LINE 29
                            // cameFrom[neighbor] := current
                            came_from[neighbor.location] = std::make_tuple<>(current.location, neighbor.action, neighbor.cost, tentative_g_score);

                            // A* LINE 30
                            // g_score[neighbor] := tentative_gScore
                            // A* LINE 31
                            // f_score[neighbor] := tentative_gScore + h(neighbor)
                            // A* LINE 32
                            // open_set.update(neighbor)
                            (*neighbor_handle).g_score = tentative_g_score;
                            (*neighbor_handle).f_score = tentative_g_score + calculate_h(neighbor.location);

                            open_set.increase(neighbor_handle);
                        }
                    }
                }
            }
        }

        // A* LINE 33
        // Open set is empty but goal was never reached
        return false;
    }
};


#endif //A_STAR_ISOLATED_HPP
