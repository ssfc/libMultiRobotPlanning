//
// Created by take_ on 2024/2/19.
//

#ifndef CBS_TA_ISOLATED_HPP
#define CBS_TA_ISOLATED_HPP

#include <map>

#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "util.hpp"

#include "next_best_assignment.hpp"
#include "timer.hpp"
#include "shortest_path_heuristic.hpp"

using libMultiRobotPlanning::NextBestAssignment;


enum class Action
{
    Up,
    Down,
    Left,
    Right,
    Wait,
};


class State
{
public:
    int time;
    int x;
    int y;

public:
    State(int input_time, int input_x, int input_y)
     : time(input_time),
       x(input_x),
       y(input_y)
    {}

    bool operator==(const State& s) const
    {
        return time == s.time && x == s.x && y == s.y;
    }

    bool equalExceptTime(const State& s) const
    {
        return x == s.x && y == s.y;
    }

    friend std::ostream& operator<<(std::ostream& os, const State& s)
    {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
        // return os << "(" << s.x << "," << s.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<State>
    {
        size_t operator()(const State& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);

            return seed;
        }
    };
}  // namespace std


struct Neighbor
{
    //! neighboring location
    State location;
    //! action to get to the neighboring location
    Action action;
    //! cost to get to the neighboring location, usually 1
    int cost;

    Neighbor(const State& input_location, const Action& input_action, int input_cost)
        : location(input_location),
          action(input_action),
          cost(input_cost)
    {}
};


struct PlanResult
{
    // path constructing locations and their g_score
    std::vector<std::pair<State, int> > path;
    //! actions and their cost
    std::vector<std::pair<Action, int> > actions;
    //! actual cost of the result
    int cost;
};

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
                return os << conflict.time << ": Edge(" << conflict.x1 << "," << conflict.y1 << "," << conflict.x2
                          << "," << conflict.y2 << ")";
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
     : time(input_time),
       x(input_x),
       y(input_y)
    {}

    bool operator<(const VertexConstraint& other) const
    {
        return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
    }

    bool operator==(const VertexConstraint& other) const
    {
        return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& vc)
    {
        return os << "VC(" << vc.time << "," << vc.x << "," << vc.y << ")";
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
}  // namespace std

class EdgeConstraint
{
public:
    int time;
    int x1;
    int y1;
    int x2;
    int y2;

public:
    EdgeConstraint(int input_time, int input_x1, int input_y1, int input_x2, int input_y2)
        : time(input_time),
       x1(input_x1),
       y1(input_y1),
       x2(input_x2),
       y2(input_y2)
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

    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& ec)
    {
        return os << "EC(" << ec.time << "," << ec.x1 << "," << ec.y1 << "," << ec.x2
                  << "," << ec.y2 << ")";
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
}  // namespace std

class Constraints
{
public:
    std::unordered_set<VertexConstraint> vertex_constraints;
    std::unordered_set<EdgeConstraint> edge_constraints;

public:
    void add(const Constraints& other)
    {
        vertex_constraints.insert(other.vertex_constraints.begin(),
                                 other.vertex_constraints.end());
        edge_constraints.insert(other.edge_constraints.begin(),
                               other.edge_constraints.end());
    }

    bool overlap(const Constraints& other) const
    {
        for (const auto& vc : vertex_constraints)
        {
            if (other.vertex_constraints.count(vc) > 0)
            {
                return true;
            }
        }

        for (const auto& ec : edge_constraints)
        {
            if (other.edge_constraints.count(ec) > 0)
            {
                return true;
            }
        }

        return false;
    }

    friend std::ostream& operator<<(std::ostream& os, const Constraints& c)
    {
        for (const auto& vc : c.vertex_constraints)
        {
            os << vc << std::endl;
        }

        for (const auto& ec : c.edge_constraints)
        {
            os << ec << std::endl;
        }

        return os;
    }
};


// inner class definition
class LowLevelNode
{
public:
    State location;
    int f_score;
    int g_score;

    // 定义 handle: 就是上面那个HeapHandle
    typename boost::heap::fibonacci_heap<LowLevelNode>::handle_type handle;
    // typename boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

public:
    LowLevelNode(const State& input_state, int input_fScore, int input_gScore)
        : location(input_state),
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
        os << "location: " << node.location << " f_score: " << node.f_score
           << " g_score: " << node.g_score;

        return os;
    }

};


class HighLevelNode
{
public:
    std::vector<PlanResult> solution;
    std::vector<Constraints> all_agents_constraints;
    std::map<size_t, Location> tasks; // maps from index to task (and does not contain an entry if no task was assigned)

    int cost;
    bool is_root;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type handle;

public:
    bool operator<(const HighLevelNode& n) const
    {
        return cost > n.cost;
    }

    Location* task(size_t idx)
    {
        Location* task = nullptr;
        auto iter = tasks.find(idx);
        if (iter != tasks.end())
        {
            task = &iter->second;
        }

        return task;
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
                os << "  " << high_level_node.solution[i].path[t].first << std::endl;
            }
            os << " Constraints:" << std::endl;
            os << high_level_node.all_agents_constraints[i];
            os << " cost: " << high_level_node.solution[i].cost << std::endl;
        }

        return os;
    }
};


class CBSTA
{
private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    size_t agent_index;
    const Location* goal;
    const Constraints* agent_constraints;
    int last_goal_constraint;
    NextBestAssignment<size_t, Location> m_assignment;
    size_t m_maxTaskAssignments;
    size_t num_task_assignments;
    int num_expanded_high_level_nodes;
    int num_expanded_low_level_nodes;
    ShortestPathHeuristic heuristic_value;
    size_t num_agents;
    std::unordered_set<Location> m_goals;

    using OpenSet = boost::heap::fibonacci_heap<LowLevelNode>;
    using HeapHandle = typename OpenSet::handle_type;
    // using OpenSet = boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
    // using HeapHandle = typename OpenSet::handle_type;

public:
    CBSTA(size_t dimx, size_t dimy,
                const std::unordered_set<Location>& obstacles,
                const std::vector<State>& startStates,
                const std::vector<std::unordered_set<Location> >& goals,
                size_t maxTaskAssignments)
        : num_columns(dimx),
          num_rows(dimy),
          obstacles(obstacles),
          agent_index(0),
          goal(nullptr),
          agent_constraints(nullptr),
          last_goal_constraint(-1),
          m_maxTaskAssignments(maxTaskAssignments),
          num_task_assignments(0),
          num_expanded_high_level_nodes(0),
          num_expanded_low_level_nodes(0),
          heuristic_value(dimx, dimy, obstacles)
    {
        num_agents = startStates.size();
        for (size_t i = 0; i < startStates.size(); ++i)
        {
            for (const auto& this_goal : goals[i])
            {
                m_assignment.setCost(i, this_goal, heuristic_value.getValue(
                                 Location(startStates[i].x, startStates[i].y), this_goal));
                m_goals.insert(this_goal);
            }
        }

        m_assignment.solve();
    }

    void set_low_level_context(size_t input_agentIdx, const Constraints* input_constraints, const Location* task)
    {
        assert(input_constraints);
        agent_index = input_agentIdx;
        goal = task;
        agent_constraints = input_constraints;
        last_goal_constraint = -1;
        if (goal != nullptr)
        {
            for (const auto& vc : input_constraints->vertex_constraints)
            {
                if (vc.x == goal->x && vc.y == goal->y)
                {
                    last_goal_constraint = std::max(last_goal_constraint, vc.time);
                }
            }
        }
        else
        {
            for (const auto& vc : input_constraints->vertex_constraints)
            {
                last_goal_constraint = std::max(last_goal_constraint, vc.time);
            }
        }
        // std::cout << "setLLCtx: " << agentIdx << " " << last_goal_constraint <<
        // std::endl;
    }

    int admissible_heuristic(const State& s)
    {
        if (goal != nullptr)
        {
            return heuristic_value.getValue(Location(s.x, s.y), *goal);
        }
        else
        {
            return 0;
        }
    }

    bool is_solution(const State& s)
    {
        bool at_goal = true;
        if (goal != nullptr)
        {
            at_goal = s.x == goal->x && s.y == goal->y;
        }

        return at_goal && s.time > last_goal_constraint;
    }

    bool location_valid(const State& s)
    {
        assert(agent_constraints);
        const auto& con = agent_constraints->vertex_constraints;

        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
               obstacles.find(Location(s.x, s.y)) == obstacles.end() &&
               con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
    }

    bool transition_valid(const State& s1, const State& s2)
    {
        assert(agent_constraints);
        const auto& con = agent_constraints->edge_constraints;

        return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) == con.end();
    }

    void get_neighbors(const State& s, std::vector<Neighbor>& neighbors)
    {
        // std::cout << "#VC " << input_constraints.vertex_constraints.size() << std::endl;
        // for(const auto& vc : input_constraints.vertex_constraints)
        // {
        //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
        //   std::endl;
        // }
        neighbors.clear();

        {
            State n(s.time + 1, s.x, s.y);
            if (location_valid(n) && transition_valid(s, n))
            {
                bool at_goal = true;

                if (goal != nullptr)
                {
                    at_goal = s.x == goal->x && s.y == goal->y;
                }

                neighbors.emplace_back(Neighbor(n, Action::Wait, at_goal ? 0 : 1));
            }
        }

        {
            State n(s.time + 1, s.x - 1, s.y);
            if (location_valid(n) && transition_valid(s, n))
            {
                neighbors.emplace_back(Neighbor(n, Action::Left, 1));
            }
        }

        {
            State n(s.time + 1, s.x + 1, s.y);
            if (location_valid(n) && transition_valid(s, n))
            {
                neighbors.emplace_back(Neighbor(n, Action::Right, 1));
            }
        }

        {
            State n(s.time + 1, s.x, s.y + 1);
            if (location_valid(n) && transition_valid(s, n))
            {
                neighbors.emplace_back(Neighbor(n, Action::Up, 1));
            }
        }

        {
            State n(s.time + 1, s.x, s.y - 1);
            if (location_valid(n) && transition_valid(s, n))
            {
                neighbors.emplace_back(Neighbor(n, Action::Down, 1));
            }
        }
    }

    bool get_first_conflict(const std::vector<PlanResult>& solution, Conflict& result)
    {
        int max_t = 0;
        for (const auto& sol : solution)
        {
            max_t = std::max<int>(max_t, sol.path.size());
        }

        for (int t = 0; t < max_t; ++t)
        {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i)
            {
                State state1 = get_state(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    State state2 = get_state(j, solution, t);
                    if (state1.equalExceptTime(state2))
                    {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Vertex;
                        result.x1 = state1.x;
                        result.y1 = state1.y;
                        // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
                        // std::endl;

                        return true;
                    }
                }
            }

            // drive-drive edge (swap)
            for (size_t i = 0; i < solution.size(); ++i)
            {
                State state1a = get_state(i, solution, t);
                State state1b = get_state(i, solution, t + 1);
                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    State state2a = get_state(j, solution, t);
                    State state2b = get_state(j, solution, t + 1);
                    if (state1a.equalExceptTime(state2b) && state1b.equalExceptTime(state2a))
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

    void generate_constraints_from_conflict(const Conflict& conflict, std::map<size_t, Constraints>& input_constraints)
    {
        if (conflict.type == Conflict::Vertex)
        {
            Constraints c1;
            c1.vertex_constraints.emplace(
                VertexConstraint(conflict.time, conflict.x1, conflict.y1));
            input_constraints[conflict.agent1] = c1;
            input_constraints[conflict.agent2] = c1;
        }
        else if (conflict.type == Conflict::Edge)
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

    void nextTaskAssignment(std::map<size_t, Location>& tasks)
    {
        if (num_task_assignments > m_maxTaskAssignments)
        {
            return;
        }

        int64_t cost = m_assignment.nextSolution(tasks);
        if (!tasks.empty())
        {
            std::cout << "nextTaskAssignment: cost: " << cost << std::endl;
            for (const auto& s : tasks)
            {
                std::cout << s.first << "->" << s.second << std::endl;
            }

            ++num_task_assignments;
        }
    }

    int get_num_expanded_high_level_nodes()
    {
        return num_expanded_high_level_nodes;
    }

    int get_num_expanded_low_level_nodes() const
    {
        return num_expanded_low_level_nodes;
    }

    size_t numTaskAssignments() const
    {
        return num_task_assignments;
    }

    State get_state(size_t agentIdx, const std::vector<PlanResult>& solution, size_t t)
    {
        assert(agentIdx < solution.size());
        if (t < solution[agentIdx].path.size())
        {
            return solution[agentIdx].path[t].first;
        }

        assert(!solution[agentIdx].path.empty());

        return solution[agentIdx].path.back().first;
    }

    bool low_level_search(const State& start_location, PlanResult& solution, int initialCost = 0)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start_location, 0));
        solution.actions.clear();
        solution.cost = 0;

        OpenSet open_set;
        std::unordered_map<State, HeapHandle, std::hash<State>> location_to_heap;
        std::unordered_set<State, std::hash<State>> closed_set;
        std::unordered_map<State, std::tuple<State,Action,int,int>,std::hash<State>> came_from;

        auto handle = open_set.push(LowLevelNode(start_location,
                                              admissible_heuristic(start_location), initialCost));
        location_to_heap.insert(std::make_pair<>(start_location, handle));
        (*handle).handle = handle;

        std::vector<Neighbor> neighbors;
        neighbors.reserve(10);

        while (!open_set.empty())
        {
            LowLevelNode current = open_set.top();
            num_expanded_low_level_nodes++;

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

                return true;
            }

            open_set.pop();
            location_to_heap.erase(current.location);
            closed_set.insert(current.location);

            // traverse neighbors
            neighbors.clear();
            get_neighbors(current.location, neighbors);
            for (const Neighbor& neighbor : neighbors)
            {
                if (closed_set.find(neighbor.location) == closed_set.end())
                {
                    int tentative_gScore = current.g_score + neighbor.cost;
                    auto iter = location_to_heap.find(neighbor.location);
                    if (iter == location_to_heap.end())
                    {  // Discover a new node
                        int f_score = tentative_gScore + admissible_heuristic(neighbor.location);
                        auto handle = open_set.push(LowLevelNode(neighbor.location, f_score, tentative_gScore));
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
                        int delta = (*handle).g_score - tentative_gScore;
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


    // Algorithm 1: high-level of CBS-TA
    // Input: Graph, start and goal locations, assignment matrix
    // Result: optimal path for each agent
    bool cbsta_search(const std::vector<State>& initialStates, std::vector<PlanResult>& solution)
    {
        HighLevelNode root;
        size_t num_agents = initialStates.size();
        root.solution.resize(num_agents);
        // A1 LINE 1
        // root.constraints ← ∅
        root.all_agents_constraints.resize(num_agents);
        root.cost = 0;

        // A1 LINE 2
        // root.assignment ← first_assignment()
        nextTaskAssignment(root.tasks);

        // A1 LINE 3
        // root.is_root ← True
        root.is_root = true;

        // A1 LINE 4
        // root.solution ← find individual paths using low-level()
        // A1 LINE 5
        // root.cost ← SIC(root.solution)
        for (size_t i = 0; i < initialStates.size(); ++i)
        {
            // if (   i < solution.size()
            //     && solution[i].path.size() > 1) {
            //   root.solution[i] = solution[i];
            //   std::cout << "use existing solution for agent: " << i << std::endl;
            // } else {
            bool success = false;
            if (!root.tasks.empty())
            {
                set_low_level_context(i, &root.all_agents_constraints[i], root.task(i));
                success = low_level_search(initialStates[i], root.solution[i]);
            }

            if (!success)
            {
                return false;
            }
            // }
            root.cost += root.solution[i].cost;
        }

        // std::priority_queue<HighLevelNode> open;
        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                         boost::heap::mutable_<true> > open;

        // A1 LINE 6
        // insert root to OPEN
        auto handle = open.push(root);
        (*handle).handle = handle;

        solution.clear();

        // A1 LINE 7
        // while OPEN not empty do
        while (!open.empty())
        {
            // A1 LINE 8
            // best_node ← best node from OPEN // lowest solution cost
            HighLevelNode best_node = open.top();
            num_expanded_high_level_nodes++;
            // std::cout << "expand: " << best_node << std::endl;

            open.pop();

            // A1 LINE 9
            // Validate the paths in best_node until a conflict occurs.
            Conflict conflict;
            // A1 LINE 10
            // if best_node has no conflict then
            // A1 LINE 20
            // Conflict ← (ai, aj, v, t) first conflict in best_node
            if (!get_first_conflict(best_node.solution, conflict))
            {
                // A1 LINE 11
                // return best_node.solution // best_node is goal
                std::cout << "done; cost: " << best_node.cost << std::endl;
                solution = best_node.solution;

                return true;
            }

            // A1 LINE 12
            // if best_node.is_root is True then
            if (best_node.is_root)
            {
                // std::cout << "root node expanded; add new root" << std::endl;

                // A1 LINE 13
                // root ← new node
                HighLevelNode new_node;
                // A1 LINE 15
                // root.assignment ← next_assignment()
                nextTaskAssignment(new_node.tasks);

                if (new_node.tasks.size() > 0)
                {
                    // A1 LINE 14
                    // root.constraints ← ∅
                    new_node.all_agents_constraints.resize(num_agents);
                    // A1 LINE 16
                    // root.is_root ← True
                    new_node.is_root = true;
                    new_node.solution.resize(num_agents);
                    new_node.cost = 0;

                    // A1 LINE 17
                    // root.solution ← find individual paths using low-level()
                    // A1 LINE 18
                    // root.cost ← SIC(root.solution)
                    bool allSuccessful = true;
                    for (size_t i = 0; i < num_agents; ++i)
                    {
                        set_low_level_context(i, &new_node.all_agents_constraints[i], new_node.task(i));
                        bool success = low_level_search(initialStates[i], new_node.solution[i]);
                        if (!success)
                        {
                            allSuccessful = false;
                            break;
                        }

                        new_node.cost += new_node.solution[i].cost;
                    }

                    if (allSuccessful)
                    {
                        // A1 LINE 19
                        // insert root to OPEN
                        auto handle = open.push(new_node);
                        (*handle).handle = handle;
                        std::cout << " new root added! cost: " << new_node.cost << std::endl;
                    }
                }
            }

            // create additional nodes to resolve conflict
            // std::cout << "Found conflict: " << conflict << std::endl;
            // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
            // conflict.type << std::endl;

            std::map<size_t, Constraints> new_constraints;
            generate_constraints_from_conflict(conflict, new_constraints);
            // A1 LINE 21
            // for agent ai in Conflict do
            for (const auto& c : new_constraints)
            {
                // std::cout << "Add HL node for " << c.first << std::endl;
                size_t i = c.first;
                HighLevelNode new_node = best_node;
                // (optional) check that this constraint was not included already
                // std::cout << new_node.all_agents_constraints[i] << std::endl;
                // std::cout << c.second << std::endl;
                assert(!new_node.all_agents_constraints[i].overlap(c.second));

                new_node.all_agents_constraints[i].add(c.second);

                new_node.cost -= new_node.solution[i].cost;

                set_low_level_context(i, &new_node.all_agents_constraints[i], new_node.task(i));
                bool success = low_level_search(initialStates[i], new_node.solution[i]);

                new_node.cost += new_node.solution[i].cost;

                if (success)
                {
                    // std::cout << "  success. cost: " << new_node.cost << std::endl;
                    auto handle = open.push(new_node);
                    (*handle).handle = handle;
                }
            }
        }

        return false;
    }
};


#endif  // CBS_TA_ISOLATED_HPP
