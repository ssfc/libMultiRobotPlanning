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

#include "neighbor.hpp"
#include "planresult.hpp"
#include "util.hpp"

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

// #include "low_level.hpp"


// Location Custom state for the search
class TimeLocation
{
public:
    int time;
    int x;
    int y;

public:
    TimeLocation(int time, int x, int y) :
            time(time), x(x), y(y)
    {}

    bool operator==(const TimeLocation& other) const
    {
        return time == other.time
               && x == other.x
               && y == other.y;
    }

    bool equal_except_time(const TimeLocation& other) const
    {
        return x == other.x && y == other.y;
    }

    friend std::ostream& operator<<(std::ostream& os, const TimeLocation& s)
    {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
        // return os << "(" << s.x << "," << s.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<TimeLocation>
    {
        size_t operator()(const TimeLocation& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);

            return seed;
        }
    };
}

// Action Custom action for the search.
// 枚举类
enum class Action
{
    Up,
    Down,
    Left,
    Right,
    Wait,
};

// Conflict Custom conflict description.
// A conflict needs to be able to be transformed into a constraint.
class Conflict
{
public:
    enum Type
    {
        Vertex,
        Edge,
    };

    int time;
    size_t agent1;
    size_t agent2;
    Type type;

    int x1;
    int y1;
    int x2;
    int y2;

public:
    friend std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
    {
        switch (conflict.type)
        {
            case Vertex:
                return os << conflict.time << ": Vertex(" << conflict.x1 << "," << conflict.y1 << ")";
            case Edge:
                return os << conflict.time << ": Edge(" << conflict.x1 << "," << conflict.y1 << ","
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
    VertexConstraint(int time, int x, int y)
            : time(time), x(x), y(y)
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
        size_t operator()(const VertexConstraint& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}


class EdgeConstraint
{
public:
    int time;
    int x1;
    int y1;
    int x2;
    int y2;

public:
    EdgeConstraint(int time, int x1, int y1, int x2, int y2)
            : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}

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
        size_t operator()(const EdgeConstraint& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x1);
            boost::hash_combine(seed, s.y1);
            boost::hash_combine(seed, s.x2);
            boost::hash_combine(seed, s.y2);

            return seed;
        }
    };
}


class Constraints
{
public:
    std::unordered_set<VertexConstraint> vertexConstraints;
    std::unordered_set<EdgeConstraint> edgeConstraints;

public:
    void add(const Constraints& other)
    {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }

    bool overlap(const Constraints& other) const
    {
        for (const auto& vc : vertexConstraints)
        {
            if (other.vertexConstraints.count(vc) > 0)
            {
                return true;
            }
        }

        for (const auto& ec : edgeConstraints)
        {
            if (other.edgeConstraints.count(ec) > 0)
            {
                return true;
            }
        }

        return false;
    }

    friend std::ostream& operator<<(std::ostream& os, const Constraints& constraints)
    {
        for (const auto& vc : constraints.vertexConstraints)
        {
            os << vc << std::endl;
        }

        for (const auto& ec : constraints.edgeConstraints)
        {
            os << ec << std::endl;
        }

        return os;
    }
};

class Environment
{
public:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    std::vector<Location> goals;
    // vector< vector<int> > m_heuristic;
    size_t agent_index;
    Constraints constraints;
    int last_goal_constraint;
    int num_expanded_high_level_nodes;
    int num_expanded_low_level_nodes;
    bool disappear_at_goal;

public:
    Environment(size_t input_dimx, size_t input_dimy, std::unordered_set<Location> input_obstacles,
                std::vector<Location> input_goals, bool input_disappearAtGoal = false)
            : num_columns(input_dimx),
              num_rows(input_dimy),
              obstacles(std::move(input_obstacles)),
              goals(std::move(input_goals)),
              agent_index(0),
            // constraints(nullptr),
              last_goal_constraint(-1),
              num_expanded_high_level_nodes(0),
              num_expanded_low_level_nodes(0),
              disappear_at_goal(input_disappearAtGoal)
    {}

    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;

    // Set the current context to a particular agent with the given set of constraints
    void set_low_Level_context(size_t agentIdx, Constraints input_constraints)
    {
        // assert(input_constraints);  // NOLINT
        agent_index = agentIdx;
        constraints = input_constraints;
        last_goal_constraint = -1;
        for (const auto& vc : input_constraints.vertexConstraints)
        {
            if (vc.x == goals[agent_index].x && vc.y == goals[agent_index].y)
            {
                last_goal_constraint = std::max(last_goal_constraint, vc.time);
            }
        }
    }

    int admissible_heuristic(const TimeLocation& time_location)
    {
        // cout << "H: " <<  s << " " << m_heuristic[agent_index][s.x + num_columns *
        // s.y] << endl;
        // return m_heuristic[agent_index][s.x + num_columns * s.y];
        return abs(time_location.x - goals[agent_index].x) +
               abs(time_location.y - goals[agent_index].y);
    }

    bool is_solution(const TimeLocation& time_location)
    {
        return time_location.x == goals[agent_index].x
               && time_location.y == goals[agent_index].y
               && time_location.time > last_goal_constraint;
    }

    void get_neighbors(const TimeLocation& time_location,
                       std::vector<Neighbor<TimeLocation, Action, int> >& neighbors)
    {
        // cout << "#VC " << constraints.vertexConstraints.size() << endl;
        // for(const auto& vc : constraints.vertexConstraints) {
        //   cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
        //   endl;
        // }
        neighbors.clear();

        TimeLocation wait_neighbor(time_location.time + 1, time_location.x, time_location.y);
        if (location_valid(wait_neighbor) && transition_valid(time_location, wait_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(wait_neighbor, Action::Wait, 1));
        }

        TimeLocation west_neighbor(time_location.time + 1, time_location.x - 1, time_location.y);
        if (location_valid(west_neighbor) && transition_valid(time_location, west_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(west_neighbor, Action::Left, 1));
        }

        TimeLocation east_neighbor(time_location.time + 1, time_location.x + 1, time_location.y);
        if (location_valid(east_neighbor) && transition_valid(time_location, east_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(east_neighbor, Action::Right, 1));
        }

        TimeLocation north_neighbor(time_location.time + 1, time_location.x, time_location.y + 1);
        if (location_valid(north_neighbor) && transition_valid(time_location, north_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(north_neighbor, Action::Up, 1));
        }

        TimeLocation south_neighbor(time_location.time + 1, time_location.x, time_location.y - 1);
        if (location_valid(south_neighbor) && transition_valid(time_location, south_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(south_neighbor, Action::Down, 1));
        }
    }

    // Finds the first conflict for the given solution for each agent.
    // Return true if a conflict was found and false otherwise.
    bool get_first_conflict(const std::vector<PlanResult<TimeLocation, Action, int> >& solution,
                          Conflict& result)
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
                        result.type = Conflict::Vertex;
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
                        result.type = Conflict::Edge;
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

    // Create a list of constraints for the given conflict.
    void generate_constraints_from_conflict(const Conflict& conflict, std::map<size_t, Constraints>& input_constraints)
    {
        if (conflict.type == Conflict::Vertex)
        {
            Constraints c1;
            c1.vertexConstraints.emplace(
                    VertexConstraint(conflict.time, conflict.x1, conflict.y1));
            input_constraints[conflict.agent1] = c1;
            input_constraints[conflict.agent2] = c1;
        }
        else if (conflict.type == Conflict::Edge)
        {
            Constraints c1;
            c1.edgeConstraints.emplace(EdgeConstraint(
                    conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
            input_constraints[conflict.agent1] = c1;
            Constraints c2;
            c2.edgeConstraints.emplace(EdgeConstraint(
                    conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
            input_constraints[conflict.agent2] = c2;
        }
    }


    TimeLocation get_time_location(size_t agentIdx,
                          const std::vector<PlanResult<TimeLocation, Action, int> >& solution,
                          size_t t)
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

    bool location_valid(const TimeLocation& s)
    {
        // assert(constraints);
        const auto& con = constraints.vertexConstraints;

        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
               obstacles.find(Location(s.x, s.y)) == obstacles.end() &&
               con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
    }

    bool transition_valid(const TimeLocation& s1, const TimeLocation& s2)
    {
        // assert(constraints);
        const auto& con = constraints.edgeConstraints;

        return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) == con.end();
    }
};

class HighLevelNode
{
public:
    std::vector<PlanResult<TimeLocation, Action, int> > solution;
    std::vector<Constraints> constraints;
    int cost;
    int id;
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
    boost::heap::mutable_<true> >::handle_type handle;

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

class LowLevelEnvironment
{
private:
    Environment& environment;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    std::vector<Location> goals;
    // vector< vector<int> > m_heuristic;
    size_t agent_index;
    Constraints constraints;
    int last_goal_constraint;
    int num_expanded_high_level_nodes;
    int num_expanded_low_level_nodes;
    bool disappear_at_goal;

public:
    LowLevelEnvironment(Environment& input_environment, size_t agentIdx,
                        const Constraints& input_constraints)
            : environment(input_environment),
            agent_index(agentIdx),
            constraints(input_constraints)
    {
        environment.set_low_Level_context(agentIdx, constraints);

        num_columns = environment.num_columns;
        num_rows = environment.num_rows;
        obstacles = environment.obstacles;
        goals = environment.goals;
        last_goal_constraint = environment.last_goal_constraint;
        num_expanded_high_level_nodes = environment.num_expanded_high_level_nodes;
        num_expanded_low_level_nodes = environment.num_expanded_low_level_nodes;
        disappear_at_goal = environment.disappear_at_goal;
    }

    int admissible_heuristic(const TimeLocation& s)
    {
        return environment.admissible_heuristic(s);
    }

    bool is_solution(const TimeLocation& s)
    {
        return environment.is_solution(s);
    }

    void get_neighbors(const TimeLocation& s, std::vector<Neighbor<TimeLocation, Action, int> >& neighbors)
    {
        environment.get_neighbors(s, neighbors);
    }

    void onExpandLowLevelNode(const TimeLocation& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        environment.num_expanded_low_level_nodes++;
    }

};


// inner class definition
class LowLevelListNode
{
public:
    TimeLocation location;
    int f_score;
    int g_score;

    // 定义 handle: 就是上面那个HeapHandle
    typename boost::heap::fibonacci_heap<LowLevelListNode>::handle_type handle;
    // typename boost::heap::d_ary_heap<LowLevelListNode, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

public:
    LowLevelListNode(const TimeLocation& input_state, int input_fScore, int input_gScore)
            : location(input_state),
              f_score(input_fScore),
              g_score(input_gScore)
    {}

    bool operator<(const LowLevelListNode& other) const
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

    friend std::ostream& operator<<(std::ostream& os, const LowLevelListNode& node)
    {
        os << "location: " << node.location << " f_score: " << node.f_score
           << " g_score: " << node.g_score;

        return os;
    }

};


class LowLevel
{
private:
    // member vars
    LowLevelEnvironment& low_level_environment; // include map size, obstacle position, agent goal.
    // 定义openSet_t和fibHeapHandle_t
    using OpenSet = boost::heap::fibonacci_heap<LowLevelListNode>;
    using HeapHandle = typename OpenSet::handle_type;
    // using OpenSet = boost::heap::d_ary_heap<LowLevelListNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
    // using HeapHandle = typename OpenSet::handle_type;

public:
    // member funcs
    LowLevel(LowLevelEnvironment& input_environment) : low_level_environment(input_environment) {}

    void onDiscover(const TimeLocation& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        // std::cout << "LL discover: " << s << std::endl;
        // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

    bool low_level_search(const TimeLocation& start_location, PlanResult<TimeLocation, Action, int>& solution,
                       int initialCost = 0)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start_location, 0));
        solution.actions.clear();
        solution.cost = 0;

        OpenSet open_set;
        std::unordered_map<TimeLocation, HeapHandle, std::hash<TimeLocation>> location_to_heap;
        std::unordered_set<TimeLocation, std::hash<TimeLocation>> closed_set;
        std::unordered_map<TimeLocation, std::tuple<TimeLocation,Action,int,int>,std::hash<TimeLocation>> came_from;

        auto handle = open_set.push(LowLevelListNode(start_location,
          low_level_environment.admissible_heuristic(start_location),
          initialCost));
        location_to_heap.insert(std::make_pair<>(start_location, handle));
        (*handle).handle = handle;

        std::vector<Neighbor<TimeLocation, Action, int> > neighbors;
        neighbors.reserve(10);

        while (!open_set.empty())
        {
            LowLevelListNode current = open_set.top();
            low_level_environment.onExpandLowLevelNode(current.location, current.f_score, current.g_score);

            if (low_level_environment.is_solution(current.location))
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
            low_level_environment.get_neighbors(current.location, neighbors);
            for (const Neighbor<TimeLocation, Action, int>& neighbor : neighbors)
            {
                if (closed_set.find(neighbor.location) == closed_set.end())
                {
                    int tentative_gScore = current.g_score + neighbor.cost;
                    auto iter = location_to_heap.find(neighbor.location);
                    if (iter == location_to_heap.end())
                    {  // Discover a new node
                        int f_score = tentative_gScore + low_level_environment.admissible_heuristic(neighbor.location);
                        auto handle = open_set.push(LowLevelListNode(neighbor.location, f_score, tentative_gScore));
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
                    // default c'tors of TimeLocation and Action are required
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



/*!
  \example cbs.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right actions
*/

/*! \brief Conflict-Based-Search (CBS) algorithm to solve the Multi-Agent
Path-Finding (MAPF) problem

This class implements the Conflict-Based-Search (CBS) algorithm.
This algorithm can find collision-free path for multiple agents with start and
goal locations given for each agent.
CBS is a two-level search. On the low-level, A* is used to find paths for
individual agents (ideally using a perfect heuristic).
The high-level is a tree-search that resolves conflicts between agents as they
occur, earliest conflict-time first.
CBS is optimal with respect to the sum-of-individual costs.

Details of the algorithm can be found in the following paper:\n
Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant:\n
"Conflict-based search for optimal multi-agent pathfinding". Artif. Intell. 219:
40-66 (2015)\n
https://doi.org/10.1016/j.artint.2014.11.006

The underlying A* can either use a fibonacci heap, or a d-ary heap.
The latter is the default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap
instead.

\tparam Location Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:

  - `Cost admissible_heuristic(const Location& s)`\n
    Admissible heuristic. Needs to take current context into account.

  - `bool is_solution(const Location& s)`\n
    Return true if the given state is a goal state for the current agent.

  - `void get_neighbors(const Location& s, std::vector<Neighbor<Location, Action, int>
>& neighbors)`\n
    Fill the list of neighboring state for the given state s and the current
agent.

*/
class HighLevel
{
private:
    Environment& environment;
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    std::vector<Location> goals;
    // vector< vector<int> > m_heuristic;
    size_t agent_index;
    Constraints constraints;
    int last_goal_constraint;
    int num_expanded_high_level_nodes;
    int num_expanded_low_level_nodes;
    bool disappear_at_goal;

    // debug var
    double start_time;

public:
    HighLevel(Environment& environment, size_t input_dimx, size_t input_dimy, std::unordered_set<Location> input_obstacles,
        std::vector<Location> input_goals, bool input_disappearAtGoal = false)
    : environment(environment),
      num_columns(input_dimx),
      num_rows(input_dimy),
      obstacles(std::move(input_obstacles)),
      goals(std::move(input_goals)),
      agent_index(0),
            // constraints(nullptr),
      last_goal_constraint(-1),
      num_expanded_high_level_nodes(0),
      num_expanded_low_level_nodes(0),
      disappear_at_goal(input_disappearAtGoal),
      start_time(clock())
    {}

    // This function is called on every high-level expansion and can be used for statistical purposes.
    void expand_high_level_node()
    {
        num_expanded_high_level_nodes++;
    }

    bool high_level_search(const std::vector<TimeLocation>& initialStates,
                           std::vector<PlanResult<TimeLocation, Action, int> >& solution)
    {
        HighLevelNode start;
        start.solution.resize(initialStates.size());
        start.constraints.resize(initialStates.size());
        start.cost = 0;
        start.id = 0;

        for (size_t i = 0; i < initialStates.size(); ++i)
        {
            // if (   i < solution.size()
            //     && solution[i].path.size() > 1) {
            //   start.solution[i] = solution[i];
            //   std::cout << "use existing solution for agent: " << i << std::endl;
            // } else {
            LowLevelEnvironment low_level_environment(environment, i, start.constraints[i]);
            LowLevel low_level(low_level_environment);
            bool is_success = low_level.low_level_search(initialStates[i], start.solution[i]);

            if (!is_success)
            {
                return false;
            }
            // }
            start.cost += start.solution[i].cost;
        }

        // std::priority_queue<HighLevelNode> open;
        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                boost::heap::mutable_<true> > open;
        auto handle = open.push(start);
        (*handle).handle = handle;

        solution.clear();
        int id = 1;
        while (!open.empty())
        {
            HighLevelNode best_node = open.top();
            expand_high_level_node();
            // std::cout << "expand: " << best_node << std::endl;

            open.pop();

            Conflict conflict;
            if (!environment.get_first_conflict(best_node.solution, conflict))
            {
                solution = best_node.solution;

                std::cout << "Planning successful! " << std::endl;

                int makespan = 0;
                for (const auto& s : solution)
                {
                    makespan = std::max<int>(makespan, s.cost);
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

                fout << "lowLevelExpanded: " << environment.num_expanded_low_level_nodes << std::endl;
                std::cerr << "lowLevelExpanded: " << environment.num_expanded_low_level_nodes << std::endl;

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
            environment.generate_constraints_from_conflict(conflict, constraints);
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
                assert(!new_node.constraints[i].overlap(constraint.second));

                new_node.constraints[i].add(constraint.second);

                new_node.cost -= new_node.solution[i].cost;

                LowLevelEnvironment low_level_environment(environment, i, new_node.constraints[i]);
                LowLevel low_level(low_level_environment);
                bool is_success = low_level.low_level_search(initialStates[i], new_node.solution[i]);

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
