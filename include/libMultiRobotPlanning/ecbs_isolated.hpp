//
// Created by take_ on 2023/12/14.
//

#ifndef ECBS_ISOLATED_HPP
#define ECBS_ISOLATED_HPP

#include <map>


#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "util.hpp"


///
enum class Action
{
    Up,
    Down,
    Left,
    Right,
    Wait,
};

class Neighbor
{
public:
    //! neighboring location
    TimeLocation time_location;
    //! action to get to the neighboring location
    Action action;
    //! cost to get to the neighboring location, usually 1
    int cost;

public:
    Neighbor(const TimeLocation& input_time_location, const Action& input_action, int input_cost)
            : time_location(input_time_location),
              action(input_action),
              cost(input_cost)
    {}
};

struct PlanResult
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


class Conflict
{
public:
    enum Type
    {
        Vertex,
        Edge,
    };

    Type type;

    int time;
    size_t agent1;
    size_t agent2;

    int x1;
    int y1;
    int x2;
    int y2;

public:
    friend std::ostream& operator<<(std::ostream& os, const Conflict& c)
    {
        switch (c.type)
        {
            case Vertex:
                return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
            case Edge:
                return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                          << "," << c.y2 << ")";
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
    VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}

    bool operator<(const VertexConstraint& other) const
    {
        return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
    }

    bool operator==(const VertexConstraint& other) const
    {
        return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c)
    {
        return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
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

    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c)
    {
        return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
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

struct Constraints
{
    std::unordered_set<VertexConstraint> vertexConstraints;
    std::unordered_set<EdgeConstraint> edgeConstraints;

    void add(const Constraints& other)
    {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }

    friend std::ostream& operator<<(std::ostream& os, const Constraints& c)
    {
        for (const auto& vc : c.vertexConstraints)
        {
            os << vc << std::endl;
        }

        for (const auto& ec : c.edgeConstraints)
        {
            os << ec << std::endl;
        }

        return os;
    }
};


class HighLevelNode
{
public:
    std::vector<PlanResult> solution;
    std::vector<Constraints> constraints;

    int cost;
    int LB;  // sum of fmin of solution

    int focal_heuristic;

    using openSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;
    using handle_t = typename openSet_t::handle_type;
    handle_t handle;

public:
    bool operator<(const HighLevelNode& n) const
    {
        // if (cost != n.cost)
        return cost > n.cost;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c)
    {
        os << " cost: " << c.cost << " LB: " << c.LB
           << " focal: " << c.focal_heuristic << std::endl;
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


class ECBSEnvironment
{
public:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    std::vector<Location> goals;
    size_t m_agentIdx;
    const Constraints* m_constraints;
    int m_lastGoalConstraint;
    int num_expanded_high_level_nodes;
    int num_expanded_low_level_nodes;
    bool m_disappearAtGoal;

    std::vector<PlanResult> m_solution;
    float factor_w;

    using openSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;
    using handle_t = typename openSet_t::handle_type;

    struct compare_focal_heuristic
    {
        bool operator()(const handle_t& h1, const handle_t& h2) const
        {
            // Our heap is a maximum heap, so we invert the comperator function here
            if ((*h1).focal_heuristic != (*h2).focal_heuristic)
            {
                return (*h1).focal_heuristic > (*h2).focal_heuristic;
            }
            else
            {
                return (*h1).cost > (*h2).cost;
            }
        }
    };

    using focalSet_t = typename boost::heap::d_ary_heap<handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compare_focal_heuristic> > ;

public:
    ECBSEnvironment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
                    std::vector<Location> input_goals, bool disappearAtGoal, float input_w)
            : num_columns(dimx),
              num_rows(dimy),
              obstacles(std::move(obstacles)),
              goals(std::move(input_goals)),
              m_agentIdx(0),
              m_constraints(nullptr),
              m_lastGoalConstraint(-1),
              num_expanded_high_level_nodes(0),
              num_expanded_low_level_nodes(0),
              m_disappearAtGoal(disappearAtGoal),
              factor_w(input_w)
    {
    }

    ECBSEnvironment(const ECBSEnvironment&) = delete;
    ECBSEnvironment& operator=(const ECBSEnvironment&) = delete;

    // low-level
    int get_num_edge_conflicts(const TimeLocation& s1a, const TimeLocation& s1b,
            const std::vector<PlanResult>& solution)
    {
        int num_conflicts = 0;
        for (size_t i = 0; i < solution.size(); ++i)
        {
            if (i != m_agentIdx && !solution[i].path.empty())
            {
                TimeLocation s2a = get_time_location(i, solution, s1a.time_step);
                TimeLocation s2b = get_time_location(i, solution, s1b.time_step);
                if ((s1a.location==s2b.location) && (s1b.location == s2a.location))
                {
                    ++num_conflicts;
                }
            }
        }

        return num_conflicts;
    }

    // Count all conflicts
    int get_focal_heuristic(const std::vector<PlanResult>& solution)
    {
        int num_conflicts = 0;

        int max_t = 0;
        for (const auto& sol : solution)
        {
            max_t = std::max<int>(max_t, sol.path.size() - 1);
        }

        for (int t = 0; t < max_t; ++t)
        {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i)
            {
                TimeLocation state1 = get_time_location(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    TimeLocation state2 = get_time_location(j, solution, t);
                    if (state1.location == state2.location)
                    {
                        ++num_conflicts;
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

                    if (state1a.location == state2b.location &&
                        state1b.location == state2a.location)
                    {
                        ++num_conflicts;
                    }
                }
            }
        }

        return num_conflicts;
    }

    bool get_all_paths_first_conflict(const std::vector<PlanResult>& solution, Conflict& result)
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
                    if (state1.location == state2.location)
                    {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Vertex;
                        result.x1 = state1.location.x;
                        result.y1 = state1.location.y;
                        // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
                        // std::endl;
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
                    if (state1a.location == state2b.location &&
                        state1b.location == state2a.location)
                    {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Edge;
                        result.x1 = state1a.location.x;
                        result.y1 = state1a.location.y;
                        result.x2 = state1b.location.x;
                        result.y2 = state1b.location.y;

                        return true;
                    }
                }
            }
        }

        return false;
    }

    void generate_constraints_from_conflict(
            const Conflict& conflict, std::map<size_t, Constraints>& constraints)
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

    void onExpandHighLevelNode()
    {
        num_expanded_high_level_nodes++;
    }

    void onExpandLowLevelNode()
    {
        num_expanded_low_level_nodes++;
    }

    int highLevelExpanded()
    {
        return num_expanded_high_level_nodes;
    }

    int lowLevelExpanded() const
    {
        return num_expanded_low_level_nodes;
    }

    TimeLocation get_time_location(size_t agentIdx, const std::vector<PlanResult>& solution, size_t t)
    {
        assert(agentIdx < solution.size());
        if (t < solution[agentIdx].path.size())
        {
            return solution[agentIdx].path[t].first;
        }

        assert(!solution[agentIdx].path.empty());
        if (m_disappearAtGoal)
        {
            // This is a trick to avoid changing the rest of the code significantly
            // After an agent disappeared, put it at a unique but invalid position
            // This will cause all calls to equalExceptTime(.) to return false.
            return TimeLocation(-1, Location(-1 * (agentIdx+1), -1));
        }

        return solution[agentIdx].path.back().first;
    }
};


struct LowLevelNode
{
    TimeLocation state;

    int fScore;
    int gScore;
    int focal_heuristic;

    using openSet_t = typename boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;
    using fibHeapHandle_t = typename openSet_t::handle_type;

    fibHeapHandle_t handle;

    LowLevelNode(const TimeLocation& input_state, int input_fScore, int input_gScore, int input_focalHeuristic)
            : state(input_state),
              fScore(input_fScore),
              gScore(input_gScore),
              focal_heuristic(input_focalHeuristic)
    {}

    bool operator<(const LowLevelNode& other) const
    {
        // Sort order
        // 1. lowest fScore
        // 2. highest gScore

        // Our heap is a maximum heap, so we invert the comperator function here
        if (fScore != other.fScore)
        {
            return fScore > other.fScore;
        }
        else
        {
            return gScore < other.gScore;
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const LowLevelNode& node)
    {
        os << "state: " << node.state << " fScore: " << node.fScore
           << " gScore: " << node.gScore << " focal: " << node.focal_heuristic;

        return os;
    }
};


class LowLevel
{
private:
    ECBSEnvironment& m_env;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
    const std::vector<PlanResult>& m_solution;
    float factor_w;

    using openSet_t = typename boost::heap::d_ary_heap<LowLevelNode, boost::heap::arity<2>,
    boost::heap::mutable_<true> >;

    using fibHeapHandle_t = typename openSet_t::handle_type;

    struct compare_focal_heuristic
    {
        bool operator()(const fibHeapHandle_t& h1, const fibHeapHandle_t& h2) const
        {
            // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
            // Path Finding" by Cohen et. al.)
            // 1. lowest focal_heuristic
            // 2. lowest fScore
            // 3. highest gScore

            // Our heap is a maximum heap, so we invert the comperator function here
            if ((*h1).focal_heuristic != (*h2).focal_heuristic)
            {
                return (*h1).focal_heuristic > (*h2).focal_heuristic;
                // } else if ((*h1).fScore != (*h2).fScore) {
                //   return (*h1).fScore > (*h2).fScore;
            }
            else if ((*h1).fScore != (*h2).fScore)
            {
                return (*h1).fScore > (*h2).fScore;
            }
            else
            {
                return (*h1).gScore < (*h2).gScore;
            }
        }
    };

    using focalSet_t = typename boost::heap::d_ary_heap<fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compare_focal_heuristic> >;

public:
    LowLevel(ECBSEnvironment& env, size_t agentIdx, const Constraints& constraints,
             const std::vector<PlanResult>& solution, float input_factor_w)
            : m_env(env)
            // , m_agentIdx(agentIdx)
            // , m_constraints(constraints)
            ,m_solution(solution),
              factor_w(input_factor_w)
    {
        set_low_level_context(agentIdx, &constraints);
    }

    void set_low_level_context(size_t agentIdx, const Constraints* constraints)
    {
        m_env.m_agentIdx = agentIdx;
        m_env.m_constraints = constraints;
        m_env.m_lastGoalConstraint = -1;
        for (const auto& vc : constraints->vertexConstraints)
        {
            if (vc.x == m_env.goals[m_env.m_agentIdx].x && vc.y == m_env.goals[m_env.m_agentIdx].y)
            {
                m_env.m_lastGoalConstraint = std::max(m_env.m_lastGoalConstraint, vc.time);
            }
        }
    }

    int admissible_heuristic(const TimeLocation& s)
    {
        return std::abs(s.location.x - m_env.goals[m_env.m_agentIdx].x) +
               std::abs(s.location.y - m_env.goals[m_env.m_agentIdx].y);
    }

    int get_num_vertex_conflicts(const TimeLocation& s)
    {
        int num_vertex_conflicts = 0;
        for (size_t i = 0; i < m_solution.size(); ++i)
        {
            if (i != m_env.m_agentIdx && !m_solution[i].path.empty())
            {
                TimeLocation state2 = m_env.get_time_location(i, m_solution, s.time_step);
                if (s.location == state2.location)
                {
                    ++num_vertex_conflicts;
                }
            }
        }

        return num_vertex_conflicts;
    }

    int get_num_edge_conflicts(const TimeLocation& s1, const TimeLocation& s2)
    {
        return m_env.get_num_edge_conflicts(s1, s2, m_solution);
    }

    bool is_solution(const TimeLocation& s)
    {
        return s.location.x == m_env.goals[m_env.m_agentIdx].x
        && s.location.y == m_env.goals[m_env.m_agentIdx].y
        && s.time_step > m_env.m_lastGoalConstraint;
    }

    bool location_valid(const TimeLocation& s)
    {
        const auto& con = m_env.m_constraints->vertexConstraints;
        return s.location.x >= 0 && s.location.x < m_env.num_columns
            && s.location.y >= 0 && s.location.y < m_env.num_rows
            && m_env.obstacles.find(Location(s.location.x, s.location.y)) == m_env.obstacles.end()
            && con.find(VertexConstraint(s.time_step, s.location.x, s.location.y)) == con.end();
    }

    bool transition_valid(const TimeLocation& s1, const TimeLocation& s2)
    {
        const auto& con = m_env.m_constraints->edgeConstraints;
        return con.find(EdgeConstraint(s1.time_step, s1.location.x, s1.location.y, s2.location.x, s2.location.y)) ==
               con.end();
    }

    void get_neighbors(const TimeLocation& s, std::vector<Neighbor>& neighbors)
    {
        neighbors.clear();

        TimeLocation wait_n(s.time_step + 1, Location(s.location.x, s.location.y));
        if (location_valid(wait_n) && transition_valid(s, wait_n))
        {
            neighbors.emplace_back(Neighbor(wait_n, Action::Wait, 1));
        }

        TimeLocation west_n(s.time_step + 1, Location(s.location.x - 1, s.location.y));
        if (location_valid(west_n) && transition_valid(s, west_n))
        {
            neighbors.emplace_back(Neighbor(west_n, Action::Left, 1));
        }

        TimeLocation east_n(s.time_step + 1, Location(s.location.x + 1, s.location.y));
        if (location_valid(east_n) && transition_valid(s, east_n))
        {
            neighbors.emplace_back(Neighbor(east_n, Action::Right, 1));
        }

        TimeLocation north_n(s.time_step + 1, Location(s.location.x, s.location.y + 1));
        if (location_valid(north_n) && transition_valid(s, north_n))
        {
            neighbors.emplace_back(Neighbor(north_n, Action::Up, 1));
        }

        TimeLocation south_n(s.time_step + 1, Location(s.location.x, s.location.y - 1));
        if (location_valid(south_n) && transition_valid(s, south_n))
        {
            neighbors.emplace_back(Neighbor(south_n, Action::Down, 1));
        }
    }

    bool low_level_search(const TimeLocation& startState, PlanResult& solution)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(startState, 0));
        solution.actions.clear();
        solution.cost = 0;

        openSet_t open_set;
        focalSet_t focal_set;  // subset of open nodes that are within suboptimality bound
        std::unordered_map<TimeLocation, fibHeapHandle_t, std::hash<TimeLocation>> timelocation_to_heaphandle;
        std::unordered_set<TimeLocation, std::hash<TimeLocation>> closed_set;
        std::unordered_map<TimeLocation, std::tuple<TimeLocation, Action, int, int>, std::hash<TimeLocation>> came_from;

        auto handle = open_set.push(
                LowLevelNode(startState, admissible_heuristic(startState), 0, 0));
        timelocation_to_heaphandle.insert(std::make_pair<>(startState, handle));
        (*handle).handle = handle;

        focal_set.push(handle);

        std::vector<Neighbor> neighbors;
        neighbors.reserve(10);

        int best_f_score = (*handle).fScore;

        // std::cout << "new search" << std::endl;

        while (!open_set.empty())
        {
            int old_best_cost = best_f_score;
            best_f_score = open_set.top().fScore;
            // std::cout << "best_f_score: " << best_f_score << std::endl;
            if (best_f_score > old_best_cost)
            {
                // std::cout << "old_best_cost: " << old_best_cost << " newBestFScore:
                // " << best_f_score << std::endl;
                auto iter = open_set.ordered_begin();
                auto iterEnd = open_set.ordered_end();
                for (; iter != iterEnd; ++iter)
                {
                    int val = iter->fScore;
                    if (val > old_best_cost * factor_w && val <= best_f_score * factor_w)
                    {
                        const LowLevelNode& n = *iter;
                        focal_set.push(n.handle);
                    }

                    if (val > best_f_score * factor_w)
                    {
                        break;
                    }
                }
            }


            // check focal list/open list consistency

            auto currentHandle = focal_set.top();
            LowLevelNode current = *currentHandle;
            m_env.onExpandLowLevelNode();

            if (is_solution(current.state))
            {
                solution.path.clear();
                solution.actions.clear();
                auto iter = came_from.find(current.state);
                while (iter != came_from.end())
                {
                    solution.path.emplace_back(
                            std::make_pair<>(iter->first, std::get<3>(iter->second)));
                    solution.actions.emplace_back(std::make_pair<>(
                            std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = came_from.find(std::get<0>(iter->second));
                }

                solution.path.emplace_back(std::make_pair<>(startState, 0));
                std::reverse(solution.path.begin(), solution.path.end());
                std::reverse(solution.actions.begin(), solution.actions.end());
                solution.cost = current.gScore;
                solution.fmin = open_set.top().fScore;

                return true;
            }

            focal_set.pop();
            open_set.erase(currentHandle);
            timelocation_to_heaphandle.erase(current.state);
            closed_set.insert(current.state);

            // traverse neighbors
            neighbors.clear();
            get_neighbors(current.state, neighbors);
            for (const Neighbor& neighbor : neighbors)
            {
                if (closed_set.find(neighbor.time_location) == closed_set.end())
                {
                    int tentative_gScore = current.gScore + neighbor.cost;
                    auto iter = timelocation_to_heaphandle.find(neighbor.time_location);
                    if (iter == timelocation_to_heaphandle.end())
                    {  // Discover a new node

                        came_from.insert(std::make_pair<>(
                                neighbor.time_location,
                                std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                                  tentative_gScore)));

                        // std::cout << "  this is a new node" << std::endl;
                        int fScore =
                                tentative_gScore + admissible_heuristic(neighbor.time_location);
                        int focal_heuristic =
                                current.focal_heuristic +
                                get_num_vertex_conflicts(neighbor.time_location) +
                                get_num_edge_conflicts(current.state, neighbor.time_location);

                        auto handle = open_set.push(
                                LowLevelNode(neighbor.time_location, fScore, tentative_gScore, focal_heuristic));
                        (*handle).handle = handle;

                        if (fScore <= best_f_score * factor_w)
                        {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focal_set.push(handle);
                        }

                        timelocation_to_heaphandle.insert(std::make_pair<>(neighbor.time_location, handle));
                        // std::cout << "  this is a new node " << fScore << "," <<
                        // tentative_gScore << std::endl;
                    }
                    else
                    {
                        auto handle = iter->second;
                        // We found this node before with a better path
                        if (tentative_gScore >= (*handle).gScore)
                        {
                            continue;
                        }

                        came_from[neighbor.time_location] = std::make_tuple<>(current.state, neighbor.action, neighbor.cost, tentative_gScore);

                        int last_gScore = (*handle).gScore;
                        int last_fScore = (*handle).fScore;
                        // std::cout << "  this is an old node: " << tentative_gScore << ","
                        // << last_gScore << " " << *handle << std::endl;
                        // update f and gScore
                        int delta = last_gScore - tentative_gScore;
                        (*handle).gScore = tentative_gScore;
                        (*handle).fScore -= delta;
                        open_set.increase(handle);

                        if ((*handle).fScore <= best_f_score * factor_w && last_fScore > best_f_score * factor_w)
                        {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focal_set.push(handle);
                        }
                    }

                }
            }
        }

        return false;
    }

};


class ECBS
{
private:
    ECBSEnvironment& m_env;
    float factor_w;

    using openSet_t = typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;
    using handle_t = typename openSet_t::handle_type;

    struct compare_focal_heuristic
    {
        bool operator()(const handle_t& h1, const handle_t& h2) const
        {
            // Our heap is a maximum heap, so we invert the comperator function here
            if ((*h1).focal_heuristic != (*h2).focal_heuristic)
            {
                return (*h1).focal_heuristic > (*h2).focal_heuristic;
            }
            else
            {
                return (*h1).cost > (*h2).cost;
            }
        }
    };

    using focalSet_t = typename boost::heap::d_ary_heap<handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compare_focal_heuristic> > ;

public:
    ECBS(ECBSEnvironment& environment, float input_w)
            : m_env(environment),
              factor_w(input_w)
    {}

    bool high_level_search(const std::vector<TimeLocation>& initialStates,
                           std::vector<PlanResult>& solution)
    {
        HighLevelNode root;
        root.solution.resize(initialStates.size());
        root.constraints.resize(initialStates.size());
        root.cost = 0;
        root.LB = 0;

        for (size_t i = 0; i < initialStates.size(); ++i)
        {
            if (i < solution.size() && solution[i].path.size() > 1)
            {
                std::cout << initialStates[i] << " " << solution[i].path.front().first
                          << std::endl;
                assert(initialStates[i] == solution[i].path.front().first);
                root.solution[i] = solution[i];
                std::cout << "use existing solution for agent: " << i << std::endl;
            }
            else
            {
                LowLevel llenv(m_env, i, root.constraints[i],
                               root.solution, factor_w);
                bool success = llenv.low_level_search(initialStates[i], root.solution[i]);
                if (!success)
                {
                    return false;
                }
            }

            root.cost += root.solution[i].cost;
            root.LB += root.solution[i].fmin;
        }

        root.focal_heuristic = m_env.get_focal_heuristic(root.solution);

        // std::priority_queue<HighLevelNode> open;
        openSet_t open_set;
        focalSet_t focal_set;

        auto handle = open_set.push(root);
        (*handle).handle = handle;
        focal_set.push(handle);

        int best_cost = (*handle).cost;

        solution.clear();
        while (!open_set.empty())
        {
            // update focal list
            {
                int old_best_cost = best_cost;
                best_cost = open_set.top().cost;
                // std::cout << "best_f_score: " << best_f_score << std::endl;
                if (best_cost > old_best_cost)
                {
                    // std::cout << "old_best_cost: " << old_best_cost << " best_cost: " <<
                    // best_cost << std::endl;
                    auto iter = open_set.ordered_begin();
                    auto iterEnd = open_set.ordered_end();
                    for (; iter != iterEnd; ++iter)
                    {
                        int val = iter->cost;
                        if (val > old_best_cost * factor_w && val <= best_cost * factor_w)
                        {
                            const HighLevelNode& n = *iter;
                            focal_set.push(n.handle);
                        }

                        if (val > best_cost * factor_w)
                        {
                            break;
                        }
                    }
                }
            }

            auto h = focal_set.top();
            HighLevelNode P = *h;
            m_env.onExpandHighLevelNode();
            // std::cout << "expand: " << P << std::endl;

            focal_set.pop();
            open_set.erase(h);

            Conflict conflict;
            if (!m_env.get_all_paths_first_conflict(P.solution, conflict))
            {
                // std::cout << "done; cost: " << P.cost << std::endl;
                solution = P.solution;

                return true;
            }

            // create additional nodes to resolve conflict
            // std::cout << "Found conflict: " << conflict << std::endl;
            // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
            // conflict.type << std::endl;

            std::map<size_t, Constraints> constraints;
            m_env.generate_constraints_from_conflict(conflict, constraints);
            for (const auto& c : constraints)
            {
                // std::cout << "Add HL node for " << c.first << std::endl;
                size_t i = c.first;
                HighLevelNode new_node = P;
                // (optional) check that this constraint was not included already
                // std::cout << new_node.constraints[i] << std::endl;
                // std::cout << c.second << std::endl;

                new_node.constraints[i].add(c.second);

                new_node.cost -= new_node.solution[i].cost;
                new_node.LB -= new_node.solution[i].fmin;

                LowLevel llenv(m_env, i, new_node.constraints[i],
                               new_node.solution, factor_w);
                bool success = llenv.low_level_search(initialStates[i], new_node.solution[i]);

                new_node.cost += new_node.solution[i].cost;
                new_node.LB += new_node.solution[i].fmin;
                new_node.focal_heuristic = m_env.get_focal_heuristic(new_node.solution);

                if (success)
                {
                    // std::cout << "  success. cost: " << new_node.cost << std::endl;
                    auto handle = open_set.push(new_node);
                    (*handle).handle = handle;

                    if (new_node.cost <= best_cost * factor_w)
                    {
                        focal_set.push(handle);
                    }
                }
            }
        }

        return false;
    }
};




#endif // ECBS_ISOLATED_HPP
