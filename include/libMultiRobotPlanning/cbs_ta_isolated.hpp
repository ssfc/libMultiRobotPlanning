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


enum class Action {
    Up,
    Down,
    Left,
    Right,
    Wait,
};

struct State {
    State(int time, int x, int y) : time(time), x(x), y(y) {}

    bool operator==(const State& s) const {
        return time == s.time && x == s.x && y == s.y;
    }

    bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

    friend std::ostream& operator<<(std::ostream& os, const State& s) {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
        // return os << "(" << s.x << "," << s.y << ")";
    }

    int time;
    int x;
    int y;
};

namespace std {
template <>
struct hash<State> {
    size_t operator()(const State& s) const {
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

template <typename State, typename Action, typename Cost>
struct PlanResult
{
    // path constructing locations and their g_score
    std::vector<std::pair<State, Cost> > path;
    //! actions and their cost
    std::vector<std::pair<Action, Cost> > actions;
    //! actual cost of the result
    Cost cost;
    //! lower bound of the cost (for suboptimal solvers)
    Cost fmin;
};

struct Conflict {
    enum Type {
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

    friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
        switch (c.type) {
            case Vertex:
                return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
            case Edge:
                return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                          << "," << c.y2 << ")";
        }
        return os;
    }
};


struct VertexConstraint {
    VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
    int time;
    int x;
    int y;

    bool operator<(const VertexConstraint& other) const {
        return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
    }

    bool operator==(const VertexConstraint& other) const {
        return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
        return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
    }
};

namespace std {
template <>
struct hash<VertexConstraint> {
    size_t operator()(const VertexConstraint& s) const {
        size_t seed = 0;
        boost::hash_combine(seed, s.time);
        boost::hash_combine(seed, s.x);
        boost::hash_combine(seed, s.y);
        return seed;
    }
};
}  // namespace std

struct EdgeConstraint {
    EdgeConstraint(int time, int x1, int y1, int x2, int y2)
        : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
    int time;
    int x1;
    int y1;
    int x2;
    int y2;

    bool operator<(const EdgeConstraint& other) const {
        return std::tie(time, x1, y1, x2, y2) <
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    bool operator==(const EdgeConstraint& other) const {
        return std::tie(time, x1, y1, x2, y2) ==
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
        return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
    size_t operator()(const EdgeConstraint& s) const {
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

struct Constraints {
    std::unordered_set<VertexConstraint> vertexConstraints;
    std::unordered_set<EdgeConstraint> edgeConstraints;

    void add(const Constraints& other) {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }

    bool overlap(const Constraints& other) const {
        for (const auto& vc : vertexConstraints) {
            if (other.vertexConstraints.count(vc) > 0) {
                return true;
            }
        }
        for (const auto& ec : edgeConstraints) {
            if (other.edgeConstraints.count(ec) > 0) {
                return true;
            }
        }
        return false;
    }

    friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
        for (const auto& vc : c.vertexConstraints) {
            os << vc << std::endl;
        }
        for (const auto& ec : c.edgeConstraints) {
            os << ec << std::endl;
        }
        return os;
    }
};


class Environment
{
private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    size_t m_agentIdx;
    const Location* m_goal;
    const Constraints* m_constraints;
    int m_lastGoalConstraint;
    NextBestAssignment<size_t, Location> m_assignment;
    size_t m_maxTaskAssignments;
    size_t m_numTaskAssignments;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;
    ShortestPathHeuristic m_heuristic;
    size_t m_numAgents;
    std::unordered_set<Location> m_goals;

public:
    Environment(size_t dimx, size_t dimy,
                const std::unordered_set<Location>& obstacles,
                const std::vector<State>& startStates,
                const std::vector<std::unordered_set<Location> >& goals,
                size_t maxTaskAssignments)
        : num_columns(dimx),
          num_rows(dimy),
          obstacles(obstacles),
          m_agentIdx(0),
          m_goal(nullptr),
          m_constraints(nullptr),
          m_lastGoalConstraint(-1),
          m_maxTaskAssignments(maxTaskAssignments),
          m_numTaskAssignments(0),
          m_highLevelExpanded(0),
          m_lowLevelExpanded(0),
          m_heuristic(dimx, dimy, obstacles)
    {
        m_numAgents = startStates.size();
        for (size_t i = 0; i < startStates.size(); ++i)
        {
            for (const auto& goal : goals[i])
            {
                m_assignment.setCost(
                    i, goal, m_heuristic.getValue(
                                 Location(startStates[i].x, startStates[i].y), goal));
                m_goals.insert(goal);
            }
        }

        m_assignment.solve();
    }

    void setLowLevelContext(size_t agentIdx, const Constraints* constraints,
                            const Location* task)
    {
        assert(constraints);
        m_agentIdx = agentIdx;
        m_goal = task;
        m_constraints = constraints;
        m_lastGoalConstraint = -1;
        if (m_goal != nullptr)
        {
            for (const auto& vc : constraints->vertexConstraints)
            {
                if (vc.x == m_goal->x && vc.y == m_goal->y)
                {
                    m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
                }
            }
        }
        else
        {
            for (const auto& vc : constraints->vertexConstraints)
            {
                m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
            }
        }
        // std::cout << "setLLCtx: " << agentIdx << " " << m_lastGoalConstraint <<
        // std::endl;
    }

    int admissible_heuristic(const State& s)
    {
        if (m_goal != nullptr)
        {
            return m_heuristic.getValue(Location(s.x, s.y), *m_goal);
        }
        else
        {
            return 0;
        }
    }

    bool is_solution(const State& s)
    {
        bool atGoal = true;
        if (m_goal != nullptr)
        {
            atGoal = s.x == m_goal->x && s.y == m_goal->y;
        }

        return atGoal && s.time > m_lastGoalConstraint;
    }

    void get_neighbors(const State& s, std::vector<Neighbor>& neighbors)
    {
        // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
        // for(const auto& vc : constraints.vertexConstraints) {
        //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
        //   std::endl;
        // }
        neighbors.clear();
        {
            State n(s.time + 1, s.x, s.y);
            if (location_valid(n) && transitionValid(s, n))
            {
                bool atGoal = true;
                if (m_goal != nullptr)
                {
                    atGoal = s.x == m_goal->x && s.y == m_goal->y;
                }
                neighbors.emplace_back(Neighbor(n, Action::Wait, atGoal ? 0 : 1));
            }
        }

        {
            State n(s.time + 1, s.x - 1, s.y);
            if (location_valid(n) && transitionValid(s, n))
            {
                neighbors.emplace_back(Neighbor(n, Action::Left, 1));
            }
        }
        {
            State n(s.time + 1, s.x + 1, s.y);
            if (location_valid(n) && transitionValid(s, n))
            {
                neighbors.emplace_back(Neighbor(n, Action::Right, 1));
            }
        }
        {
            State n(s.time + 1, s.x, s.y + 1);
            if (location_valid(n) && transitionValid(s, n))
            {
                neighbors.emplace_back(Neighbor(n, Action::Up, 1));
            }
        }
        {
            State n(s.time + 1, s.x, s.y - 1);
            if (location_valid(n) && transitionValid(s, n))
            {
                neighbors.emplace_back(Neighbor(n, Action::Down, 1));
            }
        }
    }

    bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >& solution, Conflict& result)
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
                State state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    State state2 = getState(j, solution, t);
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
                State state1a = getState(i, solution, t);
                State state1b = getState(i, solution, t + 1);
                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    State state2a = getState(j, solution, t);
                    State state2b = getState(j, solution, t + 1);
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

    void createConstraintsFromConflict(const Conflict& conflict, std::map<size_t, Constraints>& constraints)
    {
        if (conflict.type == Conflict::Vertex)
        {
            Constraints c1;
            c1.vertexConstraints.emplace(
                VertexConstraint(conflict.time, conflict.x1, conflict.y1));
            constraints[conflict.agent1] = c1;
            constraints[conflict.agent2] = c1;
        }
        else if (conflict.type == Conflict::Edge)
        {
            Constraints c1;
            c1.edgeConstraints.emplace(EdgeConstraint(
                conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
            constraints[conflict.agent1] = c1;
            Constraints c2;
            c2.edgeConstraints.emplace(EdgeConstraint(
                conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
            constraints[conflict.agent2] = c2;
        }
    }

    void nextTaskAssignment(std::map<size_t, Location>& tasks)
    {
        if (m_numTaskAssignments > m_maxTaskAssignments)
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

            ++m_numTaskAssignments;
        }
    }

    void onExpandHighLevelNode(int /*cost*/)
    {
        m_highLevelExpanded++;
    }

    void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        m_lowLevelExpanded++;
    }

    int highLevelExpanded()
    {
        return m_highLevelExpanded;
    }

    int lowLevelExpanded() const
    {
        return m_lowLevelExpanded;
    }

    size_t numTaskAssignments() const
    {
        return m_numTaskAssignments;
    }

    State getState(size_t agentIdx,
                   const std::vector<PlanResult<State, Action, int> >& solution,
                   size_t t)
    {
        assert(agentIdx < solution.size());
        if (t < solution[agentIdx].path.size())
        {
            return solution[agentIdx].path[t].first;
        }

        assert(!solution[agentIdx].path.empty());

        return solution[agentIdx].path.back().first;
    }

    bool location_valid(const State& s)
    {
        assert(m_constraints);
        const auto& con = m_constraints->vertexConstraints;

        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
               obstacles.find(Location(s.x, s.y)) == obstacles.end() &&
               con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
    }

    bool transitionValid(const State& s1, const State& s2)
    {
        assert(m_constraints);
        const auto& con = m_constraints->edgeConstraints;

        return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) == con.end();
    }
};


struct HighLevelNode
{
    std::vector<PlanResult<State, Action, int> > solution;
    std::vector<Constraints> constraints;
    std::map<size_t, Location> tasks; // maps from index to task (and does not contain an entry if no task was assigned)

    int cost;

    int id;
    bool isRoot;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type handle;

    bool operator<(const HighLevelNode& n) const
    {
        // if (cost != n.cost)
        return cost > n.cost;
        // return id > n.id;
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

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c)
    {
        os << "id: " << c.id << " cost: " << c.cost << std::endl;
        for (size_t i = 0; i < c.solution.size(); ++i)
        {
            os << "Agent: " << i << std::endl;
            os << " States:" << std::endl;
            for (size_t t = 0; t < c.solution[i].path.size(); ++t)
            {
                os << "  " << c.solution[i].path[t].first << std::endl;
            }
            os << " Constraints:" << std::endl;
            os << c.constraints[i];
            os << " cost: " << c.solution[i].cost << std::endl;
        }

        return os;
    }
};


class LowLevelEnvironment
{
private:
    Environment& m_env;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;

public:
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints, const Location* task)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
        m_env.setLowLevelContext(agentIdx, &constraints, task);
    }

    int admissible_heuristic(const State& s)
    {
        return m_env.admissible_heuristic(s);
    }

    bool is_solution(const State& s)
    {
        return m_env.is_solution(s);
    }

    void get_neighbors(const State& s, std::vector<Neighbor>& neighbors)
    {
        m_env.get_neighbors(s, neighbors);
    }

    void onExpandNode(const State& s, int fScore, int gScore)
    {
        // std::cout << "LL expand: " << s << std::endl;
        m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        // std::cout << "LL discover: " << s << std::endl;
        // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }
};


template <typename Location, typename Action, typename Environment,
          typename LocationHasher = std::hash<Location> >
class AStar
{
   private:
    // inner class declaration.
    class AStarNode;

    // member vars
    Environment& environment; // include map size, obstacle position, agent goal.
    // 定义openSet_t和fibHeapHandle_t
    using OpenSet = boost::heap::fibonacci_heap<AStarNode>;
    using HeapHandle = typename OpenSet::handle_type;
    // using OpenSet = boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
    // using HeapHandle = typename OpenSet::handle_type;

   public:
    // member funcs
    AStar(Environment& input_environment) : environment(input_environment) {}

    bool a_star_search(const Location& start_location, PlanResult<Location, Action, int>& solution,
                       int initialCost = 0)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start_location, 0));
        solution.actions.clear();
        solution.cost = 0;

        OpenSet open_set;
        std::unordered_map<Location, HeapHandle, LocationHasher> location_to_heap;
        std::unordered_set<Location, LocationHasher> closed_set;
        std::unordered_map<Location, std::tuple<Location,Action,int,int>,LocationHasher> came_from;

        auto handle = open_set.push(AStarNode(start_location,
                                              environment.admissible_heuristic(start_location), initialCost));
        location_to_heap.insert(std::make_pair<>(start_location, handle));
        (*handle).handle = handle;

        std::vector<Neighbor> neighbors;
        neighbors.reserve(10);

        while (!open_set.empty())
        {
            AStarNode current = open_set.top();
            environment.onExpandNode(current.location, current.f_score, current.g_score);

            if (environment.is_solution(current.location))
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
            for (const Neighbor& neighbor : neighbors)
            {
                if (closed_set.find(neighbor.location) == closed_set.end())
                {
                    int tentative_gScore = current.g_score + neighbor.cost;
                    auto iter = location_to_heap.find(neighbor.location);
                    if (iter == location_to_heap.end())
                    {  // Discover a new node
                        int f_score = tentative_gScore + environment.admissible_heuristic(neighbor.location);
                        auto handle = open_set.push(AStarNode(neighbor.location, f_score, tentative_gScore));
                        (*handle).handle = handle;
                        location_to_heap.insert(std::make_pair<>(neighbor.location, handle));
                        environment.onDiscover(neighbor.location, f_score, tentative_gScore);
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
                        environment.onDiscover(neighbor.location, (*handle).f_score,
                                               (*handle).g_score);
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

// inner class definition
template <typename Location, typename Action, typename Environment,
          typename StateHasher>
class AStar<Location, Action, Environment, StateHasher>::AStarNode
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



class CBSTA
{
private:
    Environment& m_env;
    typedef AStar<State, Action, LowLevelEnvironment> LowLevelSearch_t;

public:
    CBSTA(Environment& environment) : m_env(environment) {}

    bool search(const std::vector<State>& initialStates,
                std::vector<PlanResult<State, Action, int> >& solution)
    {
        HighLevelNode start;
        size_t numAgents = initialStates.size();
        start.solution.resize(numAgents);
        start.constraints.resize(numAgents);
        start.cost = 0;
        start.id = 0;
        start.isRoot = true;
        m_env.nextTaskAssignment(start.tasks);

        for (size_t i = 0; i < initialStates.size(); ++i)
        {
            // if (   i < solution.size()
            //     && solution[i].path.size() > 1) {
            //   start.solution[i] = solution[i];
            //   std::cout << "use existing solution for agent: " << i << std::endl;
            // } else {
            bool success = false;
            if (!start.tasks.empty())
            {
                LowLevelEnvironment llenv(m_env, i, start.constraints[i],
                                          start.task(i));
                LowLevelSearch_t lowLevel(llenv);
                success = lowLevel.a_star_search(initialStates[i], start.solution[i]);
            }

            if (!success)
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
            HighLevelNode P = open.top();
            m_env.onExpandHighLevelNode(P.cost);
            // std::cout << "expand: " << P << std::endl;

            open.pop();

            Conflict conflict;
            if (!m_env.getFirstConflict(P.solution, conflict))
            {
                std::cout << "done; cost: " << P.cost << std::endl;
                solution = P.solution;

                return true;
            }

            if (P.isRoot)
            {
                // std::cout << "root node expanded; add new root" << std::endl;
                HighLevelNode n;
                m_env.nextTaskAssignment(n.tasks);

                if (n.tasks.size() > 0)
                {
                    n.solution.resize(numAgents);
                    n.constraints.resize(numAgents);
                    n.cost = 0;
                    n.id = id;
                    n.isRoot = true;

                    bool allSuccessful = true;
                    for (size_t i = 0; i < numAgents; ++i)
                    {
                        LowLevelEnvironment llenv(m_env, i, n.constraints[i], n.task(i));
                        LowLevelSearch_t lowLevel(llenv);
                        bool success = lowLevel.a_star_search(initialStates[i], n.solution[i]);
                        if (!success)
                        {
                            allSuccessful = false;
                            break;
                        }

                        n.cost += n.solution[i].cost;
                    }

                    if (allSuccessful)
                    {
                        auto handle = open.push(n);
                        (*handle).handle = handle;
                        ++id;
                        std::cout << " new root added! cost: " << n.cost << std::endl;
                    }
                }
            }

            // create additional nodes to resolve conflict
            // std::cout << "Found conflict: " << conflict << std::endl;
            // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
            // conflict.type << std::endl;

            std::map<size_t, Constraints> constraints;
            m_env.createConstraintsFromConflict(conflict, constraints);
            for (const auto& c : constraints)
            {
                // std::cout << "Add HL node for " << c.first << std::endl;
                size_t i = c.first;
                // std::cout << "create child with id " << id << std::endl;
                HighLevelNode newNode = P;
                newNode.id = id;
                // (optional) check that this constraint was not included already
                // std::cout << newNode.constraints[i] << std::endl;
                // std::cout << c.second << std::endl;
                assert(!newNode.constraints[i].overlap(c.second));

                newNode.constraints[i].add(c.second);

                newNode.cost -= newNode.solution[i].cost;

                LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
                                          newNode.task(i));
                LowLevelSearch_t lowLevel(llenv);
                bool success = lowLevel.a_star_search(initialStates[i], newNode.solution[i]);

                newNode.cost += newNode.solution[i].cost;

                if (success)
                {
                    // std::cout << "  success. cost: " << newNode.cost << std::endl;
                    auto handle = open.push(newNode);
                    (*handle).handle = handle;
                }

                ++id;
            }
        }

        return false;
    }
};

#endif  // CBS_TA_ISOLATED_HPP
