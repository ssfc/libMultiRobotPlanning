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
    int calculate_h(const Location& current_location)
    {
        return abs(current_location.x - goal.x) + abs(current_location.y - goal.y);
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
        std::unordered_set<Location, std::hash<Location>> closed_set;

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

        std::vector<Child> children;
        children.reserve(10);

        // A* LINE 8
        // while openSet is not empty
        while (!open_set.empty())
        {
            // A* LINE 9
            // This operation can occur in O(Log(N)) time if open_set is a min-heap or a priority queue
            // current := the node in open_set having the lowest f_score[] value
            AStarNode current = open_set.top();
            num_expanded_nodes++;

            // A* LINE 10
            // if current = goal
            if (is_solution(current.location))
            {
                solution.path.clear();
                solution.actions.clear();
                // A* LINE 11
                // return reconstruct_path(cameFrom, current)
                // std::unordered_map<Location, std::tuple<Location,Action,int,int>,std::hash<Location>> came_from;
                auto iter = came_from.find(current.location);
                while (iter != came_from.end())
                {
                    // iter->first是location
                    // iter->second是std::tuple<Location,Action,int,int>
                    solution.path.emplace_back( // pair(Location, int)
                            std::make_pair<>(iter->first, std::get<3>(iter->second)));
                    solution.actions.emplace_back(std::make_pair<>( // pair(Location, Action)
                            std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = came_from.find(std::get<0>(iter->second)); // find(Location)
                }

                solution.path.emplace_back(std::make_pair<>(start, 0));
                std::reverse(solution.path.begin(), solution.path.end());
                std::reverse(solution.actions.begin(), solution.actions.end());
                solution.cost = current.g_score;

                std::cerr << "num expanded nodes: " << num_expanded_nodes << std::endl;
                std::cerr << "num generated nodes: " << num_generated_nodes << std::endl;

                return true;
            }

            // A* LINE 12
            // openSet.Remove(current)
            open_set.pop();
            location_to_heaphandle.erase(current.location);
            closed_set.insert(current.location);

            // traverse children
            children.clear();
            get_neighbors(current.location, children);
            // A* LINE 13
            // for each neighbor of current
            for (const Child& neighbor : children)
            {
                // If the successor has not been evaluated
                if (closed_set.find(neighbor.location) == closed_set.end()) // not in closed set
                {
                    // A* LINE 14
                    // d(current,neighbor) is the weight of the edge from current to neighbor
                    // tentative_g_score is the distance from start to the neighbor through current
                    // tentative_g_score := gScore[current] + d(current, neighbor)
                    // cost of the cheapest path from start to n currently known
                    int tentative_g_score = current.g_score + neighbor.cost;
                    auto iter = location_to_heaphandle.find(neighbor.location);
                    if (iter == location_to_heaphandle.end()) // 坐标不在堆中就不比较直接加（毕竟原g值是inf, 新g值肯定更小）
                    {  // Discover a new node
                        int f_score = tentative_g_score + calculate_h(neighbor.location);
                        auto heap_handle = open_set.emplace(AStarNode(neighbor.location, f_score, tentative_g_score));
                        location_to_heaphandle.insert(std::make_pair<>(neighbor.location, heap_handle));
                        num_generated_nodes++;
                        // std::cout << "  this is a new node " << f_score << "," <<
                        // tentative_g_score << std::endl;
                    }
                    else // 坐标在堆中。
                    {
                        auto heap_handle = iter->second; // Location所具有的其他特征。
                        // std::cout << "  this is an old node: " << tentative_g_score << ","
                        // << (*handle).g_score << std::endl;

                        // A* LINE 15
                        // if tentative_gScore < gScore[neighbor]
                        // meaning: This path to neighbor is better than any previous one. Record it!
                        // We found this node before with a better path
                        if (tentative_g_score < (*heap_handle).g_score)
                        {
                            // update f and g_score
                            (*heap_handle).g_score = tentative_g_score;
                            (*heap_handle).f_score = tentative_g_score + calculate_h(neighbor.location);

                            // A* LINE 19
                            // if neighbor not in openSet

                            // A* LINE 20
                            // openSet.add(neighbor)
                            open_set.increase(heap_handle);
                        }
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "came_from", but otherwise
                    // default c'tors of Location and Action are required
                    came_from.erase(neighbor.location);
                    // A* LINE 16
                    // cameFrom[neighbor] := current
                    // A* LINE 17
                    // gScore[neighbor] := tentative_gScore
                    // A* LINE 18
                    // fScore[neighbor] := tentative_gScore + h_score(neighbor)
                    came_from.insert(std::make_pair<>(neighbor.location,
                      std::make_tuple<>(current.location, neighbor.action, neighbor.cost, tentative_g_score)));
                }
            }
        }

        return false;
    }
};


#endif //A_STAR_ISOLATED_HPP
