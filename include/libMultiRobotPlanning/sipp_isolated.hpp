//
// Created by take_ on 2024/1/11.
//

#ifndef SIPP_ISOLATED_HPP
#define SIPP_ISOLATED_HPP

#include <boost/functional/hash.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "util.hpp"


enum class Action
{
    Up,
    Down,
    Left,
    Right,
    Wait,
};

class SIPPState
{
public:
    Location location;
    size_t interval_index;

public:
    SIPPState(const Location& input_state, size_t input_interval)
        : location(input_state),
          interval_index(input_interval)
    {}

    bool operator==(const SIPPState& other) const
    {
        return std::tie(location, interval_index) == std::tie(other.location, other.interval_index);
    }

    friend std::ostream& operator<<(std::ostream& os, const SIPPState& sipp_state)
    {
        return os << "(" << sipp_state.location << "," << sipp_state.interval_index << ")";
    }
};

struct SIPPAction
{
    SIPPAction(const Action& action, int time) : action(action), time(time) {}

    Action action;
    int time;
};

class Neighbor
{
public:
    //! neighboring location
    Location location;
    //! action to get to the neighboring location
    Action action;

public:
    Neighbor(const Location& input_location, const Action& input_action)
        : location(input_location),
          action(input_action)
    {}
};

class SIPPNeighbor
{
public:
    //! neighboring location
    SIPPState location;
    //! action to get to the neighboring location
    SIPPAction action;
    //! cost to get to the neighboring location, usually 1
    int cost;

public:
    SIPPNeighbor(const SIPPState& input_location, const SIPPAction& input_action, int input_cost)
        : location(input_location),
          action(input_action),
          cost(input_cost)
    {}
};

struct PlanResult
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

struct SIPPPlanResult
{
    // path constructing locations and their g_score
    std::vector<std::pair<SIPPState, int> > path;
    //! actions and their cost
    std::vector<std::pair<SIPPAction, int> > actions;
    //! actual cost of the result
    int cost;
    //! lower bound of the cost (for suboptimal solvers)
    int fmin;
};

class Environment
{
private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location goal;

public:
    Environment(size_t input_dimx, size_t input_dimy,
             std::unordered_set<Location> input_obstacles, Location input_goal)
        : num_columns(input_dimx),
          num_rows(input_dimy),
          obstacles(std::move(input_obstacles)),
          goal(input_goal)
    {}

    bool location_valid(const Location& location)
    {
        return location.x >= 0 && location.x < num_columns &&
               location.y >= 0 && location.y < num_rows &&
               obstacles.find(location) == obstacles.end();
    }

    float admissible_heuristic(const Location& location)
    {
        return std::abs(location.x - goal.x) + std::abs(location.y - goal.y);
    }

    bool is_solution(const Location& location)
    {
        return location == goal;
    }

    std::vector<Neighbor> get_neighbors(const Location& location)
    {
        std::vector<Neighbor> neighbors;

        Location up(location.x, location.y + 1);
        if (location_valid(up))
        {
            neighbors.emplace_back(Neighbor(up, Action::Up));
        }

        Location down(location.x, location.y - 1);
        if (location_valid(down))
        {
            neighbors.emplace_back(Neighbor(down, Action::Down));
        }

        Location left(location.x - 1, location.y);
        if (location_valid(left))
        {
            neighbors.emplace_back(Neighbor(left, Action::Left));
        }

        Location right(location.x + 1, location.y);
        if (location_valid(right))
        {
            neighbors.emplace_back(Neighbor(right, Action::Right));
        }

        return neighbors;
    }

    void onExpandNode(const Location& /*location*/, int /*fScore*/, int /*gScore*/)
    {
        // std::cout << "expand: " << location << "g: " << gScore << std::endl;
    }

    bool is_command_valid(
        const Location& /*s1*/, const Location& /*s2*/, const Action& /*a*/,
        int earliest_start_time,      // can start motion at this time
        int earliest_arrival_time,    // can only arrive at (location+cmd)
        int& t)
    {
        t = std::max<int>(earliest_arrival_time, earliest_start_time + 1);

        // TODO(whoenig): need to check for swaps here...

        // return t - 1 <= latestStartTime;
        return true;
    }
};

class Interval
{
public:
    int interval_start;
    int interval_end;

public:
    Interval(int input_start, int input_end)
        : interval_start(input_start),
          interval_end(input_end)
    {}

    friend bool operator<(const Interval& a, const Interval& b)
    {
        return a.interval_start < b.interval_start;
    }
};

class SIPPEnvironment
{
private:
    Environment m_env;
    int last_g_score;
    std::unordered_map<Location, std::vector<Interval> > location_to_safe_intervals;

public:
    SIPPEnvironment(Environment env) : m_env(env)
    {}

    int admissible_heuristic(const SIPPState& sipp_state) // 和之前的没有区别嘛
    {
        return m_env.admissible_heuristic(sipp_state.location);
    }

    // 从location_to_safe_intervals中找到对应location的safe_intervals
    const std::vector<Interval>& get_safe_intervals(const Location& location)
    {
        // [0, +inf)
        static std::vector<Interval> default_interval(1, {0, std::numeric_limits<int>::max()});
        const auto iter = location_to_safe_intervals.find(location);

        if (iter == location_to_safe_intervals.end()) // not found
        {
            return default_interval;
        }

        return iter->second;
    }

    bool is_solution(const SIPPState& sipp_state)
    {
        return m_env.is_solution(sipp_state.location) &&
               get_safe_intervals(sipp_state.location).at(sipp_state.interval_index).interval_end ==
                   std::numeric_limits<int>::max(); // 为什么goal安全区间必须是无限大的右开区间？假设goal安全区间是[4, 10], 所有智能体的行动在时刻9终结，那么不可能安全区间直到10，而必然向右延伸到无穷大。所以goal安全区间必须是无限大的右开区间。
    }

    // 这个函数对应的就是论文中的get_successors(state)
    std::vector<SIPPNeighbor> get_sipp_neighbors(const SIPPState& sipp_state)
    {
        std::vector<SIPPNeighbor> sipp_neighbors;

        std::vector<Neighbor> motions = m_env.get_neighbors(sipp_state.location);
        for (const auto& motion : motions)
        {
            // std::cout << "gN " << motion.location << std::endl;
            // std::cout << last_g_score;
            int start_t = last_g_score + 1;
            int end_t = get_safe_intervals(sipp_state.location).at(sipp_state.interval_index).interval_end;

            const auto& safe_intervals = get_safe_intervals(motion.location);
            for (size_t i = 0; i < safe_intervals.size(); ++i)
            {
                const Interval& safe_interval = safe_intervals[i];
                // std::cout << "  i " << i << ": " << safe_interval.interval_start << "," << safe_interval.interval_end <<
                // std::endl;
                if (safe_interval.interval_start - 1 > end_t || safe_interval.interval_end < start_t)
                {
                    continue;
                }

                int t;
                if (m_env.is_command_valid(sipp_state.location, motion.location, motion.action, last_g_score,
                                         safe_interval.interval_start, t))
                {
                    // std::cout << "  gN: " << motion.location << "," << i << "," << t << ","
                    // << last_g_score << std::endl;
                    sipp_neighbors.emplace_back(SIPPNeighbor(
                        SIPPState(motion.location, i),
                        SIPPAction(motion.action, 1), t - last_g_score));
                }
            }
        }

        return sipp_neighbors;
    }

    void onExpandNode(const SIPPState& sipp_state, int fScore, int gScore)
    {
        // const auto& interval_index =
        // get_safe_intervals(sipp_state.location).at(sipp_state.interval_index);
        // std::cout << "expand: " << sipp_state.location << "," << interval_index.interval_start << " to "
        // << interval_index.interval_end << "(g: " << gScore << " f: " << fScore << ")" <<
        // std::endl;
        // This is called before get_neighbors(). We use the callback to find the
        // current cost (=time) of the expanded node
        last_g_score = gScore;
        m_env.onExpandNode(sipp_state.location, fScore, gScore);
    }


    void set_collision_intervals(const Location& location, const std::vector<Interval>& intervals)
    {
        location_to_safe_intervals.erase(location);
        std::vector<Interval> sorted_intervals(intervals);
        std::sort(sorted_intervals.begin(), sorted_intervals.end());

        // std::cout << location << ": " << std::endl;
        if (intervals.size() > 0)
        {
            location_to_safe_intervals[location]; // create empty safe interval
            int start = 0;
            int last_interval_end = 0;

            for (const auto& interval : sorted_intervals)
            {
                // std::cout << "  ci: " << interval.interval_start << " - " << interval.end <<
                // std::endl;
                assert(interval.interval_start <= interval.interval_end);
                assert(start <= interval.interval_start);
                // if (start + 1 != interval.start - 1) {
                // std::cout << start << "," << interval.start << std::endl;
                // assert(start + 1 < interval.start - 1);
                if (start <= interval.interval_start - 1)
                {
                    location_to_safe_intervals[location].push_back({start, interval.interval_start - 1});
                }
                // }
                start = interval.interval_end + 1;
                last_interval_end = interval.interval_end;
            }

            if (last_interval_end < std::numeric_limits<int>::max())
            {
                // assert(start < std::numeric_limits<int>::max());
                location_to_safe_intervals[location].push_back({start, std::numeric_limits<int>::max()});
            }
        }

        // auto iter = location_to_safe_intervals.find(location);
        // if (iter != location_to_safe_intervals.interval_end()) {
        //   for (const auto& safe_interval : iter->second) {
        //     std::cout << "  safe_interval: " << safe_interval.start << " - " << safe_interval.interval_end << std::endl;
        //   }
        // }
    }

    bool find_safe_interval(const Location& location, int time, size_t& interval_index)
    {
        const auto& safe_intervals = get_safe_intervals(location);
        for (size_t idx = 0; idx < safe_intervals.size(); ++idx)
        {
            if (safe_intervals[idx].interval_start <= time && safe_intervals[idx].interval_end >= time)
            {
                interval_index = idx;

                return true;
            }
        }

        return false;
    }
};


// inner class definition
class AStarNode
{
   public:
    SIPPState location;
    int f_score;
    int g_score;

    // 定义 handle: 就是上面那个HeapHandle
    typename boost::heap::fibonacci_heap<AStarNode>::handle_type handle;
    // typename boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>>::handle_type handle;

   public:
    AStarNode(const SIPPState& input_state, int input_fScore, int input_gScore)
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

struct SIPPStateHasher
{
    size_t operator()(const SIPPState& sipp_state) const
    {
        size_t seed = 0;
        boost::hash_combine(seed, std::hash<Location>()(sipp_state.location));
        boost::hash_combine(seed, sipp_state.interval_index);

        return seed;
    }
};

class AStar
{
private:
    // inner class declaration.
    // member vars
    SIPPEnvironment& environment; // include map size, obstacle position, agent goal.
    // 定义openSet_t和fibHeapHandle_t
    using OpenSet = boost::heap::fibonacci_heap<AStarNode>;
    using HeapHandle = typename OpenSet::handle_type;
    // using OpenSet = boost::heap::d_ary_heap<AStarNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
    // using HeapHandle = typename OpenSet::handle_type;

public:
    // member funcs
    AStar(SIPPEnvironment& input_environment) : environment(input_environment)
    {}

    bool a_star_search(const SIPPState& start_location, SIPPPlanResult& sipp_solution,
                       int initialCost = 0)
    {
        sipp_solution.path.clear();
        sipp_solution.path.emplace_back(std::make_pair<>(start_location, 0));
        sipp_solution.actions.clear();
        sipp_solution.cost = 0;

        OpenSet open_set;
        std::unordered_map<SIPPState, HeapHandle, SIPPStateHasher> location_to_heap;
        std::unordered_set<SIPPState, SIPPStateHasher> closed_set;
        std::unordered_map<SIPPState, std::tuple<SIPPState,SIPPAction,int,int>,SIPPStateHasher> came_from;

        auto handle = open_set.push(AStarNode(start_location,
                                              environment.admissible_heuristic(start_location), initialCost));
        location_to_heap.insert(std::make_pair<>(start_location, handle));
        (*handle).handle = handle;

        while (!open_set.empty())
        {
            AStarNode current = open_set.top();
            environment.onExpandNode(current.location, current.f_score, current.g_score);

            if (environment.is_solution(current.location))
            {
                sipp_solution.path.clear();
                sipp_solution.actions.clear();
                auto iter = came_from.find(current.location);
                while (iter != came_from.end())
                {
                    sipp_solution.path.emplace_back(
                        std::make_pair<>(iter->first, std::get<3>(iter->second)));
                    sipp_solution.actions.emplace_back(std::make_pair<>(
                        std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = came_from.find(std::get<0>(iter->second));
                }

                sipp_solution.path.emplace_back(std::make_pair<>
                                           (start_location, initialCost));
                std::reverse(sipp_solution.path.begin(), sipp_solution.path.end());
                std::reverse(sipp_solution.actions.begin(), sipp_solution.actions.end());
                sipp_solution.cost = current.g_score;
                sipp_solution.fmin = current.f_score;

                return true;
            }

            open_set.pop();
            location_to_heap.erase(current.location);
            closed_set.insert(current.location);

            // traverse sipp_neighbors
            std::vector<SIPPNeighbor> sipp_neighbors = environment.get_sipp_neighbors(current.location);
            for (const SIPPNeighbor& sipp_neighbor : sipp_neighbors)
            {
                if (closed_set.find(sipp_neighbor.location) == closed_set.end())
                {
                    int tentative_gScore = current.g_score + sipp_neighbor.cost;
                    auto iter = location_to_heap.find(sipp_neighbor.location);
                    if (iter == location_to_heap.end())
                    {  // Discover a new node
                        int f_score = tentative_gScore + environment.admissible_heuristic(sipp_neighbor.location);
                        auto handle = open_set.push(AStarNode(sipp_neighbor.location, f_score, tentative_gScore));
                        (*handle).handle = handle;
                        location_to_heap.insert(std::make_pair<>(sipp_neighbor.location, handle));
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
                    // default c'tors of SIPPState and Action are required
                    came_from.erase(sipp_neighbor.location);
                    came_from.insert(std::make_pair<>(sipp_neighbor.location,
                      std::make_tuple<>(current.location, sipp_neighbor.action, sipp_neighbor.cost,
                                            tentative_gScore)));
                }
            }
        }

        return false;
    }
};



// #include "util.hpp"
// #include "sipp_low.hpp"


/*! \brief SIPP Algorithm to find the shortest path with dynamic obstacles

This class implements the SIPP algorithm. SIPP is an informed search algorithm
that finds the shortest path for a given map and dynamic a-priori known
obstacles. It can use a heuristic that needs to be admissible.

Details of the algorithm can be found in the following paper:\n
Mike Phillips and Maxim Likhachev:\n
"SIPP:  Safe  Interval  Path  Planning  for  Dynamic  Environments". IEEE
International Conference on Robotics and Automation (ICRA), 2011\n
https://doi.org/10.1109/ICRA.2011.5980306
*/

class SIPP
{
private:
    SIPPEnvironment m_env;
    AStar m_astar;

public:
    SIPP(SIPPEnvironment environment) :
      m_env(environment),
      m_astar(m_env)
    {}

    void set_collision_intervals(const Location& location, const std::vector<Interval>& intervals)
    {
        m_env.set_collision_intervals(location, intervals);
    }

    bool sipp_search(const Location& start, const Action& waitAction,
                PlanResult& solution, int start_time = 0)
    {
        SIPPPlanResult astar_solution;
        solution.cost = 0;
        solution.fmin = 0;
        solution.actions.clear();
        solution.path.clear();
        size_t interval;
        if (!m_env.find_safe_interval(start, start_time, interval))
        {
            return false;
        }

        bool success = m_astar.a_star_search(SIPPState(start, interval), astar_solution, start_time);
        solution.cost = astar_solution.cost - start_time;
        solution.fmin = astar_solution.fmin;

        for (size_t i = 0; i < astar_solution.actions.size(); ++i)
        {
            int wait_time = astar_solution.actions[i].second - astar_solution.actions[i].first.time;
            if (wait_time == 0)
            {
                solution.path.emplace_back(
                    std::make_pair<>(astar_solution.path[i].first.location, astar_solution.path[i].second));
                solution.actions.emplace_back(
                    std::make_pair<>(astar_solution.actions[i].first.action, astar_solution.actions[i].second));
            }
            else
            {
                // additional wait action before
                solution.path.emplace_back(
                    std::make_pair<>(astar_solution.path[i].first.location, astar_solution.path[i].second));
                solution.actions.emplace_back(std::make_pair<>(waitAction, wait_time));
                solution.path.emplace_back(
                    std::make_pair<>(astar_solution.path[i].first.location, astar_solution.path[i].second + wait_time));
                solution.actions.emplace_back(
                    std::make_pair<>(astar_solution.actions[i].first.action, astar_solution.actions[i].first.time));
            }
        }

        solution.path.emplace_back(
            std::make_pair<>(astar_solution.path.back().first.location, astar_solution.path.back().second));

        return success;
    }
};

#endif  // SIPP_ISOLATED_HPP
