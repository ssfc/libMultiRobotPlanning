//
// Created by take_ on 2023/11/7.
//

#ifndef LIBMULTIROBOTPLANNING_CBS_ISOLATED_HPP
#define LIBMULTIROBOTPLANNING_CBS_ISOLATED_HPP

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
    //! cost to get to the neighboring time_location, usually 1
    int cost;

    Neighbor(const TimeLocation& input_location, const Action& input_action, int input_cost)
            : time_location(input_location),
              action(input_action),
              cost(input_cost)
    {}
};

struct AgentPlan
{
    // path constructing locations and their g_score
    std::vector<std::pair<TimeLocation, int> > path;
    //! actions and their cost
    std::vector<std::pair<Action, int> > actions;
    //! actual cost of the result
    int cost;
    //! lower bound of the cost (for suboptimal solvers)
    int fmin;
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

    Type type;

    int time;

    size_t agent1;
    int x1; int y1;

    size_t agent2;
    int x2; int y2;

public:
    friend std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
    {
        switch (conflict.type)
        {
            case VertexConflict:
                return os << conflict.time << ": VertexConflict(" << conflict.x1 << "," << conflict.y1 << ")";
            case EdgeConflict:
                return os << conflict.time << ": EdgeConflict(" << conflict.x1 << "," << conflict.y1 << ","
                          << conflict.x2 << "," << conflict.y2 << ")";
        }

        return os;
    }
};

class VertexConstraint
{
public:
    int time;
    int x;
    int y;

public:
    VertexConstraint(int input_time, int input_x, int input_y)
            : time(input_time), x(input_x), y(input_y)
    {}

    bool operator==(const VertexConstraint& other) const
    {
        return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
    }

    bool operator<(const VertexConstraint& other) const
    {
        return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& vertex_constraint)
    {
        return os << "VC(" << vertex_constraint.time << "," << vertex_constraint.x << "," << vertex_constraint.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<VertexConstraint>
    {
        size_t operator()(const VertexConstraint& vertex_constraint) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, vertex_constraint.time);
            boost::hash_combine(seed, vertex_constraint.x);
            boost::hash_combine(seed, vertex_constraint.y);
            return seed;
        }
    };
}


class EdgeConstraint
{
public:
    int time;
    int x1; int y1;
    int x2; int y2;

public:
    EdgeConstraint(int time, int input_x1, int input_y1, int input_x2, int input_y2)
            : time(time),
            x1(input_x1), y1(input_y1),
            x2(input_x2), y2(input_y2)
            {}

    bool operator<(const EdgeConstraint& other) const
    {
        return std::tie(time, x1, y1, x2, y2) <
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    bool operator==(const EdgeConstraint& other) const
    {
        return std::tie(time, x1, y1, x2, y2) ==
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& edge_constraint)
    {
        return os << "EC(" << edge_constraint.time << "," << edge_constraint.x1 << "," << edge_constraint.y1 << "," << edge_constraint.x2
                  << "," << edge_constraint.y2 << ")";
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
            boost::hash_combine(seed, edge_constraint.time);
            boost::hash_combine(seed, edge_constraint.x1);
            boost::hash_combine(seed, edge_constraint.y1);
            boost::hash_combine(seed, edge_constraint.x2);
            boost::hash_combine(seed, edge_constraint.y2);

            return seed;
        }
    };
}


class Constraints
{
public:
    std::unordered_set<VertexConstraint> vertex_constraints;
    std::unordered_set<EdgeConstraint> edge_constraints;

public:
    // 将另一个对象中的所有元素插入到当前对象的集合中。
    void add(const Constraints& other)
    {
        vertex_constraints.insert(other.vertex_constraints.begin(),
                                 other.vertex_constraints.end());
        edge_constraints.insert(other.edge_constraints.begin(),
                               other.edge_constraints.end());
    }

    // 检查当前对象的顶点约束和边约束是否与另一个对象中的任何约束重叠。
    bool is_overlap(const Constraints& other) const
    {
        for (const auto& vertex_constraint : vertex_constraints)
        {
            if (other.vertex_constraints.count(vertex_constraint) > 0)
            {
                return true;
            }
        }

        for (const auto& edge_constraint : edge_constraints)
        {
            if (other.edge_constraints.count(edge_constraint) > 0)
            {
                return true;
            }
        }

        return false;
    }

    friend std::ostream& operator<<(std::ostream& os, const Constraints& constraints)
    {
        for (const auto& vertex_constraint : constraints.vertex_constraints)
        {
            os << vertex_constraint << std::endl;
        }

        for (const auto& edge_constraint : constraints.edge_constraints)
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
    typename boost::heap::fibonacci_heap<LowLevelNode>::handle_type handle;
    // typename boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

public:
    LowLevelNode(const TimeLocation& input_state, int input_fScore, int input_gScore)
            : time_location(input_state),
              f_score(input_fScore),
              g_score(input_gScore)
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

class HighLevelNode
{
public:
    std::vector<AgentPlan> solution;
    std::vector<Constraints> constraints;
    int cost;
    int id;
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> >::handle_type handle;
    // typename boost::heap::fibonacci_heap<HighLevelNode>::handle_type handle;

public:
    bool operator<(const HighLevelNode& other) const
    {
        // if (cost != n.cost)

        return cost > other.cost;
        // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& high_level_node)
    {
        os << "id: " << high_level_node.id << " cost: " << high_level_node.cost << std::endl;
        for (size_t i = 0; i < high_level_node.solution.size(); ++i)
        {
            os << "Agent: " << i << std::endl;
            os << " States:" << std::endl;

            for (size_t t = 0; t < high_level_node.solution[i].path.size(); ++t)
            {
                os << "  " << high_level_node.solution[i].path[t].first << std::endl;
            }

            os << " Constraints:" << std::endl;
            os << high_level_node.constraints[i];
            os << " cost: " << high_level_node.solution[i].cost << std::endl;
        }

        return os;
    }
};

class CBS
{
public:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    size_t num_agents;
    std::vector<TimeLocation> start_time_locations;
    std::vector<Location> goals;
    // vector< vector<int> > m_heuristic;
    size_t agent_index;
    Constraints constraints;
    int last_goal_constraint;
    size_t num_expanded_low_level_nodes;
    size_t num_expanded_high_level_nodes;
    bool disappear_at_goal;

    // debug var
    double start_time;

    // 定义openSet_t和fibHeapHandle_t
    using OpenHeap = boost::heap::fibonacci_heap<LowLevelNode>;
    using HeapHandle = typename OpenHeap::handle_type;
    // using OpenHeap = boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
    // using HeapHandle = typename OpenHeap::handle_type;

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
              agent_index(0),
            // constraints(nullptr),
              last_goal_constraint(-1),
              num_expanded_low_level_nodes(0),
              num_expanded_high_level_nodes(0),
              disappear_at_goal(input_disappearAtGoal),
              start_time(clock())
    {}


    // Set the current context to a particular agent with the given set of constraints
    void set_low_Level_context(size_t agentIdx, Constraints input_constraints)
    {
        // assert(input_constraints);  // NOLINT
        agent_index = agentIdx;
        constraints = input_constraints;
        last_goal_constraint = -1;
        for (const auto& vertex_constraint : input_constraints.vertex_constraints)
        {
            if (vertex_constraint.x == goals[agent_index].x && vertex_constraint.y == goals[agent_index].y)
            {
                last_goal_constraint = std::max(last_goal_constraint, vertex_constraint.time);
            }
        }
    }

    // low level 工具函数
    int admissible_heuristic(const TimeLocation& time_location)
    {
        // cout << "H: " <<  time_location << " " << m_heuristic[agent_index][time_location.x + num_columns *
        // time_location.y] << endl;
        // return m_heuristic[agent_index][time_location.x + num_columns * time_location.y];
        return abs(time_location.x - goals[agent_index].x) +
               abs(time_location.y - goals[agent_index].y);
    }

    // low level 工具函数
    bool is_solution(const TimeLocation& time_location)
    {
        return time_location.x == goals[agent_index].x
               && time_location.y == goals[agent_index].y
               && time_location.time > last_goal_constraint;
    }

    // low level 工具函数 get_neighbors的工具函数
    bool location_valid(const TimeLocation& time_location)
    {
        // assert(constraints);
        const auto& con = constraints.vertex_constraints;

        return time_location.x >= 0 && time_location.x < num_columns
               && time_location.y >= 0 && time_location.y < num_rows
               && obstacles.find(Location(time_location.x, time_location.y)) == obstacles.end()
               && con.find(VertexConstraint(time_location.time, time_location.x, time_location.y)) == con.end();
    }

    // low level 工具函数 get_neighbors的工具函数
    bool transition_valid(const TimeLocation& s1, const TimeLocation& s2)
    {
        // assert(constraints);
        const auto& con = constraints.edge_constraints;

        return con.find(EdgeConstraint(s1.time, s1.x, s1.y,
                                       s2.x, s2.y)) == con.end();
    }

    // low level 工具函数
    void get_neighbors(const TimeLocation& time_location, std::vector<Neighbor>& neighbors)
    {
        // cout << "#VC " << constraints.vertex_constraints.size() << endl;
        // for(const auto& vertex_constraint : constraints.vertex_constraints) {
        //   cout << "  " << vertex_constraint.time << "," << vertex_constraint.x << "," << vertex_constraint.y <<
        //   endl;
        // }
        neighbors.clear();

        TimeLocation wait_neighbor(time_location.time + 1, time_location.x, time_location.y);
        if (location_valid(wait_neighbor) && transition_valid(time_location, wait_neighbor))
        {
            neighbors.emplace_back(Neighbor(wait_neighbor, Action::Wait, 1));
        }

        TimeLocation west_neighbor(time_location.time + 1, time_location.x - 1, time_location.y);
        if (location_valid(west_neighbor) && transition_valid(time_location, west_neighbor))
        {
            neighbors.emplace_back(Neighbor(west_neighbor, Action::East, 1));
        }

        TimeLocation east_neighbor(time_location.time + 1, time_location.x + 1, time_location.y);
        if (location_valid(east_neighbor) && transition_valid(time_location, east_neighbor))
        {
            neighbors.emplace_back(Neighbor(east_neighbor, Action::West, 1));
        }

        TimeLocation north_neighbor(time_location.time + 1, time_location.x, time_location.y + 1);
        if (location_valid(north_neighbor) && transition_valid(time_location, north_neighbor))
        {
            neighbors.emplace_back(Neighbor(north_neighbor, Action::North, 1));
        }

        TimeLocation south_neighbor(time_location.time + 1, time_location.x, time_location.y - 1);
        if (location_valid(south_neighbor) && transition_valid(time_location, south_neighbor))
        {
            neighbors.emplace_back(Neighbor(south_neighbor, Action::South, 1));
        }
    }

    bool low_level_search(const TimeLocation& start_location, AgentPlan& solution)
    {
        int initialCost = 0;
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start_location, 0));
        solution.actions.clear();
        solution.cost = 0;

        OpenHeap open_heap;
        std::unordered_map<TimeLocation, HeapHandle, std::hash<TimeLocation>> location_to_heap;
        std::unordered_set<TimeLocation, std::hash<TimeLocation>> closed_set;
        std::unordered_map<TimeLocation, std::tuple<TimeLocation,Action,int,int>,std::hash<TimeLocation>> came_from;

        auto handle = open_heap.push(LowLevelNode(start_location,
                         admissible_heuristic(start_location),
                         initialCost));
        location_to_heap.insert(std::make_pair<>(start_location, handle));
        (*handle).handle = handle;

        std::vector<Neighbor> neighbors;
        neighbors.reserve(10);

        while (!open_heap.empty())
        {
            LowLevelNode current = open_heap.top();
            num_expanded_low_level_nodes++;

            if (is_solution(current.time_location))
            {
                solution.path.clear();
                solution.actions.clear();
                auto iter = came_from.find(current.time_location);
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

            open_heap.pop();
            location_to_heap.erase(current.time_location);
            closed_set.insert(current.time_location);

            // traverse neighbors
            neighbors.clear();
            get_neighbors(current.time_location, neighbors);
            for (const Neighbor& neighbor : neighbors)
            {
                if (closed_set.find(neighbor.time_location) == closed_set.end())
                {
                    int tentative_gScore = current.g_score + neighbor.cost;
                    auto iter = location_to_heap.find(neighbor.time_location);
                    if (iter == location_to_heap.end())
                    {  // Discover a new node
                        int f_score = tentative_gScore + admissible_heuristic(neighbor.time_location);
                        auto handle = open_heap.push(LowLevelNode(neighbor.time_location, f_score, tentative_gScore));
                        (*handle).handle = handle;
                        location_to_heap.insert(std::make_pair<>(neighbor.time_location, handle));
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
                        open_heap.increase(handle);
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "came_from", but otherwise
                    // default c'tors of TimeLocation and Action are required
                    came_from.erase(neighbor.time_location);
                    came_from.insert(std::make_pair<>(neighbor.time_location,
                                                      std::make_tuple<>(current.time_location, neighbor.action, neighbor.cost,
                                                                        tentative_gScore)));
                }
            }
        }

        return false;
    }

    // HighLevel 工具函数 get_first_conflict 的工具函数
    TimeLocation get_time_location(size_t agentIdx, const std::vector<AgentPlan>& solution, size_t t)
    {
        assert(agentIdx < solution.size());

        if (t < solution[agentIdx].path.size())
        {
            return solution[agentIdx].path[t].first;
        }

        assert(!solution[agentIdx].path.empty());

        if (disappear_at_goal)
        {
            // This is a trick to avoid changing the rest of the code significantly
            // After an agent disappeared, put it at a unique but invalid position
            // This will cause all calls to equal_except_time(.) to return false.
            return TimeLocation(-1, -1 * (agentIdx + 1), -1);
        }

        return solution[agentIdx].path.back().first;
    }

    // HighLevel 工具函数
    // Finds the first conflict for the given solution for each agent.
    // Return true if a conflict was found and false otherwise.
    bool get_first_conflict(const std::vector<AgentPlan>& solution, Conflict& result)
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
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::VertexConflict;
                        result.x1 = state1.x;
                        result.y1 = state1.y;
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
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::EdgeConflict;
                        result.x1 = state1a.x;
                        result.y1 = state1a.y;
                        result.x2 = state1b.x;
                        result.y2 = state1b.y;

                        return true;
                    }
                }
            }
        }

        return false;
    }

    // High level 工具函数
    // Create a list of constraints for the given conflict.
    void generate_constraints_from_conflict(const Conflict& conflict, std::map<size_t, Constraints>& input_constraints)
    {
        if (conflict.type == Conflict::VertexConflict)
        {
            Constraints c1;
            c1.vertex_constraints.emplace(
                    VertexConstraint(conflict.time, conflict.x1, conflict.y1));
            input_constraints[conflict.agent1] = c1;
            input_constraints[conflict.agent2] = c1;
        }
        else if (conflict.type == Conflict::EdgeConflict)
        {
            Constraints c1;
            c1.edge_constraints.emplace(EdgeConstraint(
                    conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
            input_constraints[conflict.agent1] = c1;
            Constraints c2;
            c2.edge_constraints.emplace(EdgeConstraint(
                    conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
            input_constraints[conflict.agent2] = c2;
        }
    }

    bool high_level_search()
    {
        std::vector<AgentPlan> solution;

        HighLevelNode root;
        root.solution.resize(num_agents);
        // std::cerr << "start_time_locations size: " << start_time_locations.size() << std::endl;
        // A1 LINE 1
        // Root.constraints = ∅ // 最开始无约束
        root.constraints.resize(num_agents);
        root.cost = 0;
        root.id = 0;

        // A1 LINE 2
        // Root.solution = find individual paths using the low-level() // 用低层算法计算每个智能体的path
        for (size_t i = 0; i < num_agents; i++)
        {
            set_low_Level_context(i, root.constraints[i]);
            bool is_success = low_level_search(start_time_locations[i], root.solution[i]);

            if (!is_success)
            {
                return false;
            }

            // Implement A1 LINE 3
            // Root.cost = SIC(Root.solution) // SIC: abbreviation of sum of individual costs heuristic
            root.cost += root.solution[i].cost;
        }

        // Implement A1 LINE 4
        // insert Root to OPEN
        // std::priority_queue<HighLevelNode> open;
        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> > open;
        // typename boost::heap::fibonacci_heap<HighLevelNode>::handle_type open;
        auto handle = open.push(root);
        (*handle).handle = handle;

        solution.clear();
        int id = 1;
        while (!open.empty())
        {
            HighLevelNode best_node = open.top();
            num_expanded_high_level_nodes++; // high-level node expanded
            // std::cout << "expand: " << best_node << std::endl;

            open.pop();

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

                std::ofstream fout("output.yaml");
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
                    // cout << "Solution for: " << i << endl;
                    // for (size_t i = 0; i < solution[i].actions.size(); ++i) {
                    //   cout << solution[i].path[i].second << ": " <<
                    //   solution[i].path[i].first << "->" << solution[i].actions[i].first
                    //   << "(cost: " << solution[i].actions[i].second << ")" << endl;
                    // }
                    // cout << solution[i].path.back().second << ": " <<
                    // solution[i].path.back().first << endl;

                    fout << "  agent" << i << ":" << std::endl;
                    for (const auto& state : solution[i].path)
                    {
                        fout << "    - x: " << state.first.x << std::endl
                             << "      y: " << state.first.y << std::endl
                             << "      t: " << state.second << std::endl;
                    }

                    std::cerr << "agent " << i << ": ";
                    for (const auto& state : solution[i].path)
                    {
                        std::cerr << "(" << state.first.x << "," << state.first.y << "),";
                    }
                    std::cerr << std::endl;
                }

                return true;
            }

            // create additional nodes to resolve conflict
            // std::cout << "Found conflict: " << conflict << std::endl;
            // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
            // conflict.type << std::endl;

            std::map<size_t, Constraints> constraints;
            generate_constraints_from_conflict(conflict, constraints);
            for (const auto& constraint : constraints)
            {
                // std::cout << "Add HL node for " << constraint.first << std::endl;
                size_t i = constraint.first;
                // std::cout << "create child with id " << id << std::endl;
                HighLevelNode new_node = best_node;
                new_node.id = id;
                // (optional) check that this constraint was not included already
                // std::cout << new_node.constraints[i] << std::endl;
                // std::cout << constraint.second << std::endl;
                assert(!new_node.constraints[i].is_overlap(constraint.second));

                new_node.constraints[i].add(constraint.second);

                new_node.cost -= new_node.solution[i].cost;

                // LowLevelEnvironment environment(environment, i, new_node.constraints[i]);
                set_low_Level_context(i, new_node.constraints[i]);
                bool is_success = low_level_search(start_time_locations[i], new_node.solution[i]);

                new_node.cost += new_node.solution[i].cost;

                if (is_success)
                {
                    // std::cout << "  is_success. cost: " << new_node.cost << std::endl;
                    auto handle = open.push(new_node);
                    (*handle).handle = handle;
                }

                ++id;
            }
        }

        return false;
    }
};


#endif //LIBMULTIROBOTPLANNING_CBS_ISOLATED_HPP
