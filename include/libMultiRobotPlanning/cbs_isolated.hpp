//
// Created by take_ on 2023/11/7.
//

#ifndef LIBMULTIROBOTPLANNING_CBS_ISOLATED_HPP
#define LIBMULTIROBOTPLANNING_CBS_ISOLATED_HPP

#include <filesystem>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <vector>

#include <boost/functional/hash.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include "util.hpp"

// #include "low_level.hpp"


// Action Custom action for the search.
// 枚举类
enum class Action
{
    North,
    South,
    East,
    West,
    Wait,
};

struct Neighbor
{
    //! neighboring time_location
    TimeLocation time_location;
    //! action to get to the neighboring time_location
    Action action;

    Neighbor(const TimeLocation& input_location, const Action& input_action)
            : time_location(input_location),
              action(input_action)
    {}
};

struct AgentPlan
{
    // path constructing locations and their g_score
    std::vector<TimeLocation> path;
    //! actual cost of the result
    int cost;
};

// Conflict Custom conflict description.
// A conflict needs to be able to be transformed into a constraint.
class Conflict
{
public:
    enum Type
    {
        VertexConflict,
        EdgeConflict,
    };

    Type conflict_type;

    size_t agent_id_1;
    size_t agent_id_2;

    int time_step;

    std::vector<Location> locations; // 可能是1个(点冲突), 也可能是2个(边冲突)

public:
    friend std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
    {
        switch (conflict.conflict_type)
        {
            case VertexConflict:
                return os << conflict.time_step << ": VertexConflict(" << conflict.locations[0].x
                    << "," << conflict.locations[0].y << ")";
            case EdgeConflict:
                return os << conflict.time_step << ": EdgeConflict(" << conflict.locations[0].x << "," << conflict.locations[0].y << ","
                          << conflict.locations[1].x << "," << conflict.locations[1].y << ")";
        }

        return os;
    }
};

// 由于是negative constraint, 所以记录的是禁止的state.
class NegativeConstraint
{
public:
    enum Type
    {
        VertexConstraint,
        EdgeConstraint,
    };

    Type constraint_type;

    int agent_id; // 人物
    int time_step; // 时间
    std::vector<Location> locations; // 可能是1个(点冲突), 也可能是2个(边冲突)
    // 如果发生边冲突, locations的顺序是agent的行进顺序

public:
    NegativeConstraint() = default;

    NegativeConstraint(Type input_type,
                       int input_agent_id,
                       int input_time,
                       std::vector<Location> input_locations) :
            constraint_type(input_type),
            agent_id(input_agent_id),
            time_step(input_time),
            locations(std::move(input_locations))
    {}

    bool operator==(const NegativeConstraint& other) const
    {
        return std::tie(agent_id, locations, time_step)
               == std::tie(other.agent_id, other.locations, other.time_step);
    }

    bool operator!=(const NegativeConstraint& other) const
    {
        return std::tie(agent_id, locations, time_step)
               != std::tie(other.agent_id, other.locations, other.time_step);
    }
};


class EdgeConstraint
{
public:
    int time;
    Location location_1;
    Location location_2;

public:
    EdgeConstraint(int time, Location input_location_1, Location input_location_2)
            : time(time),
              location_1(input_location_1),
              location_2(input_location_2)
    {}

    bool operator<(const EdgeConstraint& other) const
    {
        return std::tie(time, location_1.x, location_1.y, location_2.x, location_2.y) <
               std::tie(other.time, other.location_1.x, other.location_1.y, other.location_2.x, other.location_2.y);
    }

    bool operator==(const EdgeConstraint& other) const
    {
        return std::tie(time, location_1.x, location_1.y, location_2.x, location_2.y) ==
               std::tie(other.time, other.location_1.x, other.location_1.y, other.location_2.x, other.location_2.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& edge_constraint)
    {
        return os << "EC(" << edge_constraint.time << ","
                  << edge_constraint.location_1.x << "," << edge_constraint.location_1.y << ","
                  << edge_constraint.location_2.x << "," << edge_constraint.location_2.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<EdgeConstraint>
    {
        size_t operator()(const EdgeConstraint& edge_constraint) const
        {
            size_t seed = 0;

            boost::hash_combine(seed, hash<int>()(edge_constraint.time));
            boost::hash_combine(seed, hash<Location>()(edge_constraint.location_1));
            boost::hash_combine(seed, hash<Location>()(edge_constraint.location_2));

            return seed;
        }
    };
}

// constraints of a single agent
class AgentConstraints
{
public:
    std::unordered_set<TimeLocation> vertex_constraints;
    std::unordered_set<EdgeConstraint> edge_constraints;
    int max_goal_constraint_time;

public:
    // 将另一个对象中的所有元素插入到当前对象的集合中。
    void add(const NegativeConstraint& new_constraint)
    {
        if(new_constraint.constraint_type == NegativeConstraint::VertexConstraint)
        {
            vertex_constraints.insert(TimeLocation(new_constraint.time_step,
               new_constraint.locations[0]));
        }
        else
        {
            edge_constraints.insert(EdgeConstraint(new_constraint.time_step,
               new_constraint.locations[0], new_constraint.locations[1]));
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const AgentConstraints& input_constraints)
    {
        for (const auto& vertex_constraint : input_constraints.vertex_constraints)
        {
            os << vertex_constraint << std::endl;
        }

        for (const auto& edge_constraint : input_constraints.edge_constraints)
        {
            os << edge_constraint << std::endl;
        }

        return os;
    }
};


class LowLevelNode
{
public:
    TimeLocation time_location;
    int f_score;
    int g_score;

    // 定义 handle: 就是上面那个HeapHandle
    // typename boost::heap::fibonacci_heap<LowLevelNode>::handle_type handle;
    typename boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

public:
    LowLevelNode(const TimeLocation& input_state, int input_f_score, int input_g_score)
            : time_location(input_state),
              f_score(input_f_score),
              g_score(input_g_score)
    {}

    bool operator<(const LowLevelNode& other) const
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

    friend std::ostream& operator<<(std::ostream& os, const LowLevelNode& node)
    {
        os << "time_location: " << node.time_location << " f_score: " << node.f_score
           << " g_score: " << node.g_score;

        return os;
    }

};

class LowLevel
{
public:
    // input var
    size_t num_columns;
    size_t num_rows;
    std::unordered_set<Location> obstacles;

    TimeLocation start_time_location;
    Location goal;
    AgentConstraints agent_constraints;
    bool disappear_at_goal;

    // process var
    int max_goal_constraint_time;

public:
    LowLevel(size_t input_num_columns,
             size_t input_num_rows,
             std::unordered_set<Location> input_obstacles,
             TimeLocation input_time_location,
             Location input_goal,
             AgentConstraints input_constraints,
             int input_max_goal_constraint_time,
             bool input_disappear_at_goal):
            num_columns(input_num_columns),
            num_rows(input_num_rows),
            obstacles(std::move(input_obstacles)),
            start_time_location(input_time_location),
            goal(input_goal),
            agent_constraints(input_constraints),
            max_goal_constraint_time(input_max_goal_constraint_time),
            disappear_at_goal(input_disappear_at_goal)
    {
        // assert(input_constraints);  // NOLINT
        for (const auto& vertex_constraint : agent_constraints.vertex_constraints)
        {
            if (vertex_constraint.location.x == goal.x && vertex_constraint.location.y == goal.y)
            {
                max_goal_constraint_time = std::max(max_goal_constraint_time, vertex_constraint.time);
            }
        }
    }

    // Set the current context to a particular agent with the given set of agent_constraints
    void set_low_level_context(TimeLocation input_time_location,
                               Location input_goal,
                               AgentConstraints input_constraints,
                               int input_max_goal_constraint_time)
    {
        start_time_location = input_time_location;
        goal = input_goal;
        agent_constraints = input_constraints;

        // assert(input_constraints);  // NOLINT
        max_goal_constraint_time = input_max_goal_constraint_time;
        for (const auto& vertex_constraint : agent_constraints.vertex_constraints)
        {
            if (vertex_constraint.location == goal)
            {
                max_goal_constraint_time = std::max(max_goal_constraint_time, vertex_constraint.time);
            }
        }
    }

    // low level 工具函数
    int calculate_h(const Location& location)
    {
        // cout << "H: " <<  time_location << " " << m_heuristic[low_level_agent_index][time_location.x + num_columns *
        // time_location.y] << endl;
        // return m_heuristic[low_level_agent_index][time_location.x + num_columns * time_location.y];
        return abs(location.x - goal.x) + abs(location.y - goal.y);
    }

    // low level 工具函数
    bool is_solution(const TimeLocation& time_location)
    {
        return time_location.location == goal && time_location.time > max_goal_constraint_time;
        // 显然，max_goal_constraint_time越小越好。
    }

    // 函数功能
    // 判断当前位置是否在地图范围内。
    bool is_in_range(const Location& location) const
    {
        if(location.x >= 0 && location.x < num_columns
           && location.y >= 0 && location.y < num_rows)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool is_obstacle(const Location& location) const
    {
        if(obstacles.find(location) != obstacles.end())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // low level 工具函数 get_neighbors的工具函数
    bool is_element_of_vertex_constraints(const TimeLocation& time_location)
    {
        // assert(agent_constraints);
        return agent_constraints.vertex_constraints.find(time_location) != agent_constraints.vertex_constraints.end();
    }

    // low level 工具函数 get_neighbors的工具函数
    bool is_element_of_edge_constraints(const TimeLocation& s1, const TimeLocation& s2)
    {
        return agent_constraints.edge_constraints.find(EdgeConstraint(s1.time, s1.location, s2.location)) != agent_constraints.edge_constraints.end();
    }

    // low level 工具函数: 引用传递计算大型结果
    std::vector<Neighbor> get_neighbors(const TimeLocation& time_location)
    {
        // cout << "#VC " << agent_constraints.vertex_constraints.size() << endl;
        // for(const auto& vertex_constraint : agent_constraints.vertex_constraints) {
        //   cout << "  " << vertex_constraint.time << "," << vertex_constraint.x << "," << vertex_constraint.y <<
        //   endl;
        // }
        std::vector<Neighbor> neighbors;

        TimeLocation wait_neighbor(time_location.time + 1, Location(time_location.location.x, time_location.location.y));
        if (is_in_range(wait_neighbor.location)
        && !is_obstacle(wait_neighbor.location)
        && !is_element_of_vertex_constraints(wait_neighbor)
        && !is_element_of_edge_constraints(time_location, wait_neighbor))
        {
            neighbors.emplace_back(Neighbor(wait_neighbor, Action::Wait));
        }

        TimeLocation west_neighbor(time_location.time + 1, Location(time_location.location.x - 1, time_location.location.y));
        if (is_in_range(west_neighbor.location)
        && !is_obstacle(west_neighbor.location)
        && !is_element_of_vertex_constraints(west_neighbor)
        && !is_element_of_edge_constraints(time_location, west_neighbor))
        {
            neighbors.emplace_back(Neighbor(west_neighbor, Action::East));
        }

        TimeLocation east_neighbor(time_location.time + 1, Location(time_location.location.x + 1, time_location.location.y));
        if (is_in_range(east_neighbor.location)
        && !is_obstacle(east_neighbor.location)
        && !is_element_of_vertex_constraints(east_neighbor)
        && !is_element_of_edge_constraints(time_location, east_neighbor))
        {
            neighbors.emplace_back(Neighbor(east_neighbor, Action::West));
        }

        TimeLocation north_neighbor(time_location.time + 1, Location(time_location.location.x, time_location.location.y + 1));
        if (is_in_range(north_neighbor.location)
        && !is_obstacle(north_neighbor.location)
        && !is_element_of_vertex_constraints(north_neighbor)
        && !is_element_of_edge_constraints(time_location, north_neighbor))
        {
            neighbors.emplace_back(Neighbor(north_neighbor, Action::North));
        }

        TimeLocation south_neighbor(time_location.time + 1, Location(time_location.location.x, time_location.location.y - 1));
        if (is_in_range(south_neighbor.location)
        && !is_obstacle(south_neighbor.location)
        && !is_element_of_vertex_constraints(south_neighbor)
        && !is_element_of_edge_constraints(time_location, south_neighbor))
        {
            neighbors.emplace_back(Neighbor(south_neighbor, Action::South));
        }

        return neighbors;
    }

    // 引用传递大型计算结果
    // A* LINE 1
    // A* finds a path from start to goal.
    // h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    bool low_level_search(AgentPlan& solution, size_t& num_expanded_low_level_nodes)
    {
        // A* LINE 2
        // For node n, came_from[n] is the node immediately preceding it on the cheapest path from the start
        // to n currently known.
        // came_from := an empty map
        std::unordered_map<TimeLocation, std::tuple<TimeLocation,Action,int,int>,std::hash<TimeLocation>> came_from;

        // A* LINE 3
        // For node n, g_score[n] is the cost of the cheapest path from start to n currently known.
        // g_score := map with default value of Infinity

        // A* LINE 4
        // g_score[start] := 0

        // 定义openSet_t和fibHeapHandle_t
        // using OpenHeap = boost::heap::fibonacci_heap<LowLevelNode>;
        // using HeapHandle = typename OpenHeap::handle_type;
        using OpenHeap = boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
        using HeapHandle = typename OpenHeap::handle_type;

        OpenHeap open_set;
        std::unordered_map<TimeLocation, HeapHandle, std::hash<TimeLocation>> timelocation_to_heaphandle;

        // A* LINE 5
        // For node n, f_score[n] := g_score[n] + h(n). f_score[n] represents our current best guess as to
        // how cheap a path could be from start to finish if it goes through n.
        // f_score := map with default value of Infinity

        // A* LINE 6
        // f_score[start] := h(start)

        // A* LINE 7
        // The set of discovered nodes that may need to be (re-)expanded.
        // Initially, only the start node is known.
        // This is usually implemented as a min-heap or priority queue rather than a hash-set.
        // open_set := {start}
        auto handle = open_set.emplace(LowLevelNode(start_time_location,
            calculate_h(start_time_location.location), 0));
        timelocation_to_heaphandle.insert(std::make_pair<>(start_time_location, handle));

        // A* LINE 8
        // node that has already been evaluated. In other words, already been poped from open_set.
        // closed_set := the empty set
        std::unordered_set<TimeLocation, std::hash<TimeLocation>> closed_set;

        // A* LINE 9
        // while open_set is not empty
        while (!open_set.empty())
        {
            // A* LINE 10
            // This operation can occur in O(Log(N)) time if open_set is a min-heap or a priority queue
            // current := the node in open_set having the lowest f_score[] value
            auto current = open_set.top();
            num_expanded_low_level_nodes++;

            // A* LINE 11
            // if current = goal
            if (is_solution(current.time_location))
            {
                solution.path.clear();

                // A* LINE 12
                // total_path := {current}
                solution.path.emplace(solution.path.begin(), current.time_location);

                // A* LINE 13
                // while current in came_from.Keys:
                auto iter = came_from.find(current.time_location);
                while (iter != came_from.end())
                {
                    // A* LINE 14
                    // current := came_from[current]

                    // A* LINE 15
                    // total_path.prepend(current)

                    solution.path.emplace(solution.path.begin(), std::get<0>(iter->second));
                    iter = came_from.find(std::get<0>(iter->second));
                }

                solution.cost = current.g_score;

                // A* LINE 16
                // return total_path
                return true;
            }

            // A* LINE 17
            // open_set.Remove(current)
            open_set.pop();
            timelocation_to_heaphandle.erase(current.time_location);
            // A* LINE 18
            // add current to closed_set.
            closed_set.insert(current.time_location);

            // A* LINE 19
            // for each neighbor of current
            // traverse neighbors
            auto neighbors = get_neighbors(current.time_location);
            for (const auto& neighbor : neighbors)
            {
                // A* LINE 20
                // if neighbor not in closed_set
                if (closed_set.find(neighbor.time_location) == closed_set.end())
                {
                    // A* LINE 21
                    // d(current,neighbor) is the weight of the edge from current to neighbor
                    // tentative_g_score is the distance from start to the neighbor through current
                    // tentative_g_score := g_score[current] + d(current, neighbor)
                    int tentative_g_score = current.g_score + 1;

                    // A* LINE 22
                    // if neighbor not in open_set
                    auto iter = timelocation_to_heaphandle.find(neighbor.time_location);
                    if (iter == timelocation_to_heaphandle.end())
                    {
                        // A* LINE 23
                        // This path to neighbor is better than any previous one. Record it!
                        // Discover a new node
                        came_from.insert(std::make_pair<>(neighbor.time_location,
                          std::make_tuple<>(current.time_location, neighbor.action, 1, tentative_g_score)));

                        // A* LINE 24
                        // g_score[neighbor] := tentative_g_score

                        // A* LINE 25
                        // f_score[neighbor] := tentative_g_score + h(neighbor)

                        // A* LINE 26
                        // open_set.add(neighbor)
                        int f_score = tentative_g_score + calculate_h(neighbor.time_location.location);
                        auto handle = open_set.emplace(LowLevelNode(neighbor.time_location, f_score, tentative_g_score));
                        timelocation_to_heaphandle.insert(std::make_pair<>(neighbor.time_location, handle));
                        // std::cout << "  this is a new node " << f_score << "," <<
                        // tentative_g_score << std::endl;
                    }
                    // A* LINE 27
                    else
                    {
                        auto handle = iter->second;
                        // std::cout << "  this is an old node: " << tentative_g_score << ","
                        // << (*handle).g_score << std::endl;
                        // We found this node before with a better path

                        // A* LINE 28
                        // This path to neighbor is better than any previous one. Record it!
                        // if tentative_g_score < g_score[neighbor]
                        if (tentative_g_score < (*handle).g_score)
                        {
                            // A* LINE 29
                            // came_from[neighbor] := current
                            came_from[neighbor.time_location] = std::make_tuple<>(current.time_location, neighbor.action, 1, tentative_g_score);

                            // A* LINE 30
                            // g_score[neighbor] := tentative_g_score

                            // A* LINE 31
                            // f_score[neighbor] := tentative_g_score + h(neighbor)
                            // update f and g_score
                            (*handle).g_score = tentative_g_score;
                            (*handle).f_score = tentative_g_score + calculate_h(neighbor.time_location.location);

                            // A* LINE 32
                            // open_set.update(neighbor)
                            open_set.increase(handle);
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

class HighLevelNode
{
public:
    std::vector<AgentPlan> solution;
    std::vector<AgentConstraints> all_agents_constraints;
    int cost;
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> >::handle_type handle;

public:
    bool operator<(const HighLevelNode& other) const
    {
        // if (cost != n.cost)

        return cost > other.cost;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& high_level_node)
    {
        os << " cost: " << high_level_node.cost << std::endl;
        for (size_t i = 0; i < high_level_node.solution.size(); ++i)
        {
            os << "Agent: " << i << std::endl;
            os << " States:" << std::endl;

            for (size_t t = 0; t < high_level_node.solution[i].path.size(); ++t)
            {
                os << "  " << high_level_node.solution[i].path[t] << std::endl;
            }

            os << " AgentConstraints:" << std::endl;
            os << high_level_node.all_agents_constraints[i];
            os << " cost: " << high_level_node.solution[i].cost << std::endl;
        }

        return os;
    }
};

class CBS
{
public:
    // high level vars.
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    size_t num_agents;
    std::vector<TimeLocation> start_time_locations;
    std::vector<Location> goals;

    // low-level vars.
    // vector< vector<int> > m_heuristic;
    size_t num_expanded_high_level_nodes;
    size_t num_expanded_low_level_nodes;
    bool disappear_at_goal;

    // debug var
    double start_time;

public:
    CBS(size_t input_dimx, size_t input_dimy, std::unordered_set<Location> input_obstacles,
        size_t input_num_agents,
        std::vector<TimeLocation> input_start_time_locations,
        std::vector<Location> input_goals,
        bool input_disappearAtGoal = false)
            : num_columns(input_dimx),
              num_rows(input_dimy),
              obstacles(std::move(input_obstacles)),
              num_agents(input_num_agents),
              start_time_locations(std::move(input_start_time_locations)),
              goals(std::move(input_goals)),
            // agent_constraints(nullptr),
              num_expanded_high_level_nodes(0),
              num_expanded_low_level_nodes(0),
              disappear_at_goal(input_disappearAtGoal),
              start_time(clock())
    {
        // 检查地图读入是否正确。
        /*
        std::cerr<<"goals[0].x: " << goals[0].x << std::endl;
        std::cerr<<"goals[0].y: " << goals[0].y << std::endl;

        for (const auto& obstacle : obstacles)
        {
            //打印当前元素的值
            std::cout << "(" << obstacle.x << "," << obstacle.y << ")";
        }
        std::cout << std::endl;
         */
    }

    void generate_text_instance(std::string input_filename)
    {
        std::vector<std::vector<char>> map;
        map.resize(num_rows);
        for(int i=0;i<num_rows;i++)
        {
            map[i].resize(num_columns);
            for(int j=0;j<num_columns;j++)
            {
                map[i][j] = '.';
            }
        }

        for (const auto& obstacle : obstacles)
        {
            //打印当前元素的值
            map[obstacle.y][obstacle.x] = '@';
        }

        std::ofstream fout(input_filename, std::ios::out);
        fout << num_rows << " " << num_columns << std::endl;
        for(int i=0;i<num_rows;i++)
        {
            for(int j=0;j<num_columns;j++)
            {
                fout << map[i][j] << " ";
            }
            fout << std::endl;
        }

        fout << num_agents << std::endl;
        for (size_t i=0;i<num_agents;i++)
        {
            fout << start_time_locations[i].location.x << " " << start_time_locations[i].location.y << " "
                 << goals[i].x << " " << goals[i].y << std::endl;
        }

    }

    // HighLevel 工具函数 get_first_conflict 的工具函数
    TimeLocation get_time_location(size_t input_agent_index, const std::vector<AgentPlan>& solution, size_t t)
    {
        if (t < solution[input_agent_index].path.size())
        {
            return solution[input_agent_index].path[t];
        }

        if (disappear_at_goal)
        {
            // This is a trick to avoid changing the rest of the code significantly
            // After an agent disappeared, put it at a unique but invalid position
            // This will cause all calls to equal_except_time(.) to return false.
            return TimeLocation(-1, Location(-1 * (input_agent_index + 1), -1));
        }

        return solution[input_agent_index].path.back();
    }

    // HighLevel 工具函数: 引用传递计算大型结果
    // Finds the first conflict for the given solution for each agent.
    // Return true if a conflict was found and false otherwise.
    // Comment: 这个函数有两个目标，是否有冲突，有冲突还要引用并改变冲突。
    bool get_first_conflict(const std::vector<AgentPlan>& solution, Conflict& first_conflict)
    {
        int max_t = 0;
        for (const auto& sol : solution)
        {
            max_t = std::max<int>(max_t, sol.path.size() - 1);
        }

        for (int t = 0; t <= max_t; ++t)
        {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i)
            {
                TimeLocation state1 = get_time_location(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    TimeLocation state2 = get_time_location(j, solution, t);
                    if (state1.equal_except_time(state2))
                    {
                        first_conflict.time_step = t;
                        first_conflict.agent_id_1 = i;
                        first_conflict.agent_id_2 = j;
                        first_conflict.conflict_type = Conflict::VertexConflict;
                        first_conflict.locations.emplace_back(state1.location);
                        // cout << "VC " << t << "," << state1.x << "," << state1.y <<
                        // endl;

                        return true;
                    }
                }
            }

            // drive-drive edge (swap)
            for (size_t i = 0; i < solution.size(); ++i)
            {
                TimeLocation state1a = get_time_location(i, solution, t);
                TimeLocation state1b = get_time_location(i, solution, t + 1);

                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    TimeLocation state2a = get_time_location(j, solution, t);
                    TimeLocation state2b = get_time_location(j, solution, t + 1);
                    if (state1a.equal_except_time(state2b) && state1b.equal_except_time(state2a))
                    {
                        first_conflict.time_step = t;
                        first_conflict.agent_id_1 = i;
                        first_conflict.agent_id_2 = j;
                        first_conflict.conflict_type = Conflict::EdgeConflict;
                        first_conflict.locations.emplace_back(state1a.location);
                        first_conflict.locations.emplace_back(state1b.location);

                        return true;
                    }
                }
            }
        }

        return false;
    }

    // High level 工具函数
    // Create a list of agent_constraints for the given conflict.
    std::vector<NegativeConstraint> generate_constraints_from_conflict(const Conflict& input_conflict)
    {
        std::vector<NegativeConstraint> constraints_from_conflict;

        if(input_conflict.conflict_type == Conflict::VertexConflict)  // vertex conflict
        {
            constraints_from_conflict.emplace_back(NegativeConstraint::VertexConstraint,
                   input_conflict.agent_id_1, input_conflict.time_step, input_conflict.locations);
            constraints_from_conflict.emplace_back(NegativeConstraint::VertexConstraint,
                   input_conflict.agent_id_2, input_conflict.time_step, input_conflict.locations);
        }
        else  // Edge conflict
        {
            constraints_from_conflict.emplace_back(NegativeConstraint::EdgeConstraint,
                   input_conflict.agent_id_1, input_conflict.time_step, input_conflict.locations);
            std::vector<Location> reversed_locations = {input_conflict.locations[1], input_conflict.locations[0]};
            constraints_from_conflict.emplace_back(NegativeConstraint::EdgeConstraint,
                   input_conflict.agent_id_2, input_conflict.time_step, reversed_locations);
        }

        return constraints_from_conflict;
    }

    // 引用传递大型计算结果
    bool high_level_search()
    {
        std::vector<AgentPlan> solution;

        HighLevelNode root;
        root.solution.resize(num_agents);
        // std::cerr << "start_time_locations size: " << start_time_locations.size() << std::endl;
        // A1 LINE 1
        // Root.agent_constraints = ∅ // 最开始无约束
        root.all_agents_constraints.resize(num_agents);
        root.cost = 0;

        // A1 LINE 2
        // Root.solution = find individual paths using the low-level() // 用低层算法计算每个智能体的path
        auto low_level = LowLevel(num_columns, num_rows, obstacles,
                                  start_time_locations[0], goals[0],
                                  root.all_agents_constraints[0], -1, false);
        for (size_t i = 0; i < num_agents; i++)
        {
            low_level.set_low_level_context(start_time_locations[i], goals[i], root.all_agents_constraints[i], -1);
            bool is_path_found = low_level.low_level_search(root.solution[i], num_expanded_low_level_nodes);

            if (!is_path_found)
            {
                return false;
            }

            //  A1 LINE 3
            // Root.cost = SIC(Root.solution) // SIC: abbreviation of sum of individual costs heuristic
            root.cost += root.solution[i].cost;
        }

        //  A1 LINE 4
        // insert Root to OPEN
        // std::priority_queue<HighLevelNode> open;
        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> > open;
        open.emplace(root);

        //  A1 LINE 5
        // while OPEN not empty do
        while (!open.empty())
        {
            //  A1 LINE 6
            // P ← the best node from OPEN // the lowest solution cost
            HighLevelNode best_node = open.top();
            num_expanded_high_level_nodes++; // high-level node expanded
            // std::cout << "expand: " << best_node << std::endl;
            open.pop();

            //  A1 LINE 7
            // Validate the paths in P until a conflict occurs.
            //  A1 LINE 8
            // if P has no conflict then
            //  A1 LINE 10
            // C ← first conflict (ai, aj , v, t) in P
            Conflict conflict;
            if (!get_first_conflict(best_node.solution, conflict))
            {
                solution = best_node.solution;

                std::cout << "Planning successful! " << std::endl;

                int makespan = 0;
                for (const auto& agent_plan : solution)
                {
                    makespan = std::max<int>(makespan, agent_plan.cost);
                }

                std::ofstream fout("output.yaml", std::ios::out);
                fout << "statistics:" << std::endl;

                fout << "cost: " << best_node.cost << std::endl;
                std::cerr << "cost: " << best_node.cost << std::endl;

                fout << "makespan: " << makespan << std::endl;

                double elapsed_time = (clock() - start_time) / CLOCKS_PER_SEC;
                std::cerr << "runtime: " << elapsed_time * 1000 << "ms" << std::endl;

                fout << "highLevelExpanded: " << num_expanded_high_level_nodes << std::endl;
                std::cerr << "highLevelExpanded: " << num_expanded_high_level_nodes << std::endl;

                fout << "lowLevelExpanded: " << num_expanded_low_level_nodes << std::endl;
                std::cerr << "lowLevelExpanded: " << num_expanded_low_level_nodes << std::endl;

                fout << "schedule:" << std::endl;
                for (size_t i = 0; i < solution.size(); i++)
                {
                    fout << "  agent" << i << ":" << std::endl;
                    for (const auto& state : solution[i].path)
                    {
                        fout << "    - x: " << state.location.x << std::endl
                             << "      y: " << state.location.y << std::endl
                             << "      t: " << state.time << std::endl;
                    }

                    std::cerr << "agent " << i << ": ";
                    for (const auto& state : solution[i].path)
                    {
                        std::cerr << "(" << state.location.x << "," << state.location.y << "),";
                    }
                    std::cerr << std::endl;
                }

                return true;
            }

            // create additional nodes to resolve conflict
            // std::cout << "Found conflict: " << conflict << std::endl;
            // std::cout << "Found conflict at t=" << conflict.time << " conflict_type: " <<
            // conflict.conflict_type << std::endl;

            // A1 LINE 11
            // for each agent ai in C do
            auto new_constraints = generate_constraints_from_conflict(conflict);
            for (const auto& new_constraint : new_constraints)
            {
                // std::cout << "Add HL node for " << new_constraint.first << std::endl;
                size_t i = new_constraint.agent_id;
                // A1 LINE 12
                // new_node ← new node
                HighLevelNode new_node = best_node;
                // (optional) check that this new_constraint was not included already
                // std::cout << new_node.all_agents_constraints[i] << std::endl;
                // std::cout << new_constraint.second << std::endl;

                // A1 LINE 13
                // new_node.all_agents_constraints ← best_node.all_agents_constraints + (ai, s, t)
                new_node.all_agents_constraints[i].add(new_constraint);
                // 为什么这里的constraints不会和new_constraint重叠？
                // 因为low-level-search已经满足旧constraints, 所以新产生的constraint不可能和已有的constraint重叠，所以无需重叠检测。

                if (new_constraint.constraint_type == NegativeConstraint::VertexConstraint)
                {
                    new_node.all_agents_constraints[i].max_goal_constraint_time = std::max(
                        new_node.all_agents_constraints[i].max_goal_constraint_time, new_constraint.time_step);
                }

                // A1 LINE 16
                // new_node.cost = SIC(new_node.solution)
                // 这里是增量更新，计算前先减去，算完后再加回来。
                new_node.cost -= new_node.solution[i].cost;

                low_level.set_low_level_context(start_time_locations[i], goals[i], new_node.all_agents_constraints[i], new_node.all_agents_constraints[i].max_goal_constraint_time);
                bool is_path_found = low_level.low_level_search(new_node.solution[i], num_expanded_low_level_nodes);

                if (is_path_found)
                {
                    new_node.cost += new_node.solution[i].cost;
                    // std::cout << "  is_path_found. cost: " << new_node.cost << std::endl;
                    // A1 LINE 17
                    // Insert new_node to OPEN
                    open.emplace(new_node);
                }
            }
        }

        return false;
    }
};


#endif //LIBMULTIROBOTPLANNING_CBS_ISOLATED_HPP