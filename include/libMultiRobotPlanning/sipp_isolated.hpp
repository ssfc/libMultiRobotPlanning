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
    Location state;
    size_t interval;

public:
    SIPPState(const Location& input_state, size_t input_interval)
        : state(input_state),
          interval(input_interval)
    {}

    bool operator==(const SIPPState& other) const
    {
        return std::tie(state, interval) == std::tie(other.state, other.interval);
    }

    friend std::ostream& operator<<(std::ostream& os, const SIPPState& s)
    {
        return os << "(" << s.state << "," << s.interval << ")";
    }
};

struct SIPPStateHasher
{
    size_t operator()(const SIPPState& s) const
    {
        size_t seed = 0;
        boost::hash_combine(seed, std::hash<Location>()(s.state));
        boost::hash_combine(seed, s.interval);

        return seed;
    }
};

struct SIPPAction
{
    SIPPAction(const Action& action, int time) : action(action), time(time) {}

    Action action;
    int time;
};

template <typename Location, typename Action>
class Neighbor
{
public:
    //! neighboring location
    Location location;
    //! action to get to the neighboring location
    Action action;
    //! cost to get to the neighboring location, usually 1
    int cost;

public:
    Neighbor(const Location& input_location, const Action& input_action, int input_cost)
        : location(input_location),
          action(input_action),
          cost(input_cost)
    {}
};

template <typename Location, typename Action>
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

template <typename SIPPEnvironment, typename LocationHasher = std::hash<SIPPState> >
class AStar
{
private:
    // inner class declaration.
    class AStarNode;

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

    bool a_star_search(const SIPPState& start_location, PlanResult<SIPPState, SIPPAction>& solution,
                       int initialCost = 0)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start_location, 0));
        solution.actions.clear();
        solution.cost = 0;

        OpenSet open_set;
        std::unordered_map<SIPPState, HeapHandle, LocationHasher> location_to_heap;
        std::unordered_set<SIPPState, LocationHasher> closed_set;
        std::unordered_map<SIPPState, std::tuple<SIPPState,SIPPAction,int,int>,LocationHasher> came_from;

        auto handle = open_set.push(AStarNode(start_location,
                                              environment.admissible_heuristic(start_location), initialCost));
        location_to_heap.insert(std::make_pair<>(start_location, handle));
        (*handle).handle = handle;

        std::vector<Neighbor<SIPPState, SIPPAction> > neighbors;
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
            environment.get_sipp_neighbors(current.location, neighbors);
            for (const Neighbor<SIPPState, SIPPAction>& neighbor : neighbors)
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
                        environment.onDiscover(neighbor.location, (*handle).f_score, (*handle).g_score);
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "came_from", but otherwise
                    // default c'tors of SIPPState and Action are required
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
template <typename SIPPEnvironment, typename StateHasher>
class AStar<SIPPEnvironment, StateHasher>::AStarNode
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

class Environment
{
private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location m_goal;

   public:
    Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles, Location goal)
        : num_columns(dimx),
          num_rows(dimy),
          obstacles(std::move(obstacles)),
          m_goal(goal)
    {}

    bool location_valid(const Location& s)
    {
        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
               obstacles.find(s) == obstacles.end();
    }

    float admissible_heuristic(const Location& s)
    {
        return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
    }

    bool is_solution(const Location& s)
    {
        return s == m_goal;
    }

    Location getLocation(const Location& s)
    {
        return s;
    }

    void get_neighbors(const Location& s, std::vector<Neighbor<Location, Action> >& neighbors)
    {
        neighbors.clear();

        Location up(s.x, s.y + 1);
        if (location_valid(up))
        {
            neighbors.emplace_back(Neighbor<Location, Action>(up, Action::Up, 1));
        }

        Location down(s.x, s.y - 1);
        if (location_valid(down))
        {
            neighbors.emplace_back(Neighbor<Location, Action>(down, Action::Down, 1));
        }

        Location left(s.x - 1, s.y);
        if (location_valid(left))
        {
            neighbors.emplace_back(Neighbor<Location, Action>(left, Action::Left, 1));
        }

        Location right(s.x + 1, s.y);
        if (location_valid(right))
        {
            neighbors.emplace_back(Neighbor<Location, Action>(right, Action::Right, 1));
        }
    }

    void onExpandNode(const Location& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        // std::cout << "expand: " << s << "g: " << gScore << std::endl;
    }

    void onDiscover(const Location& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        // std::cout << "  discover: " << s << std::endl;
    }

    bool isCommandValid(
        const Location& /*s1*/, const Location& /*s2*/, const Action& /*a*/,
        int earliestStartTime,      // can start motion at this time
        int /*latestStartTime*/,    // must have left s by this time
        int earliestArrivalTime,    // can only arrive at (s+cmd)
        int /*latestArrivalTime*/,  // needs to arrive by this time at (s+cmd)
        int& t)
    {
        t = std::max<int>(earliestArrivalTime, earliestStartTime + 1);

        // TODO(whoenig): need to check for swaps here...

        // return t - 1 <= latestStartTime;
        return true;
    }
};

class SIPP
{
public:
    class Interval
    {
    public:
        int start;
        int end;

    public:
        Interval(int input_start, int input_end)
            : start(input_start),
              end(input_end)
        {}

        friend bool operator<(const Interval& a, const Interval& b)
        {
            return a.start < b.start;
        }
    };

public:
    SIPP(Environment& environment) : m_env(environment), m_astar(m_env)
    {}

    void setCollisionIntervals(const Location& location, const std::vector<Interval>& intervals)
    {
        m_env.setCollisionIntervals(location, intervals);
    }

    bool mightHaveSolution(const Location& goal)
    {
        return m_env.mightHaveSolution(goal);
    }

    bool sipp_search(const Location& startState, const Action& waitAction,
                PlanResult<Location, Action>& solution, int startTime = 0)
    {
        PlanResult<SIPPState, SIPPAction> astarsolution;
        solution.cost = 0;
        solution.fmin = 0;
        solution.actions.clear();
        solution.path.clear();
        size_t interval;
        if (!m_env.findSafeInterval(startState, startTime, interval))
        {
            return false;
        }

        bool success = m_astar.a_star_search(SIPPState(startState, interval), astarsolution, startTime);
        solution.cost = astarsolution.cost - startTime;
        solution.fmin = astarsolution.fmin;

        for (size_t i = 0; i < astarsolution.actions.size(); ++i)
        {
            int waitTime = astarsolution.actions[i].second - astarsolution.actions[i].first.time;
            if (waitTime == 0)
            {
                solution.path.emplace_back(
                    std::make_pair<>(astarsolution.path[i].first.state, astarsolution.path[i].second));
                solution.actions.emplace_back(
                    std::make_pair<>(astarsolution.actions[i].first.action, astarsolution.actions[i].second));
            }
            else
            {
                // additional wait action before
                solution.path.emplace_back(
                    std::make_pair<>(astarsolution.path[i].first.state, astarsolution.path[i].second));
                solution.actions.emplace_back(std::make_pair<>(waitAction, waitTime));
                solution.path.emplace_back(
                    std::make_pair<>(astarsolution.path[i].first.state, astarsolution.path[i].second + waitTime));
                solution.actions.emplace_back(
                    std::make_pair<>(astarsolution.actions[i].first.action, astarsolution.actions[i].first.time));
            }
        }

        solution.path.emplace_back(
            std::make_pair<>(astarsolution.path.back().first.state, astarsolution.path.back().second));

        return success;
    }

private:
    // private:
    struct SIPPEnvironment
    {
        SIPPEnvironment(Environment& env) : m_env(env)
        {}

        int admissible_heuristic(const SIPPState& s)
        {
            return m_env.admissible_heuristic(s.state);
        }

        bool mightHaveSolution(const Location& goal)
        {
            const auto& si = safeIntervals(m_env.getLocation(goal));
            return m_env.is_solution(goal) &&
                   !si.empty() &&
                   si.back().end == std::numeric_limits<int>::max();
        }

        bool is_solution(const SIPPState& s)
        {
            return m_env.is_solution(s.state) &&
                   safeIntervals(m_env.getLocation(s.state)).at(s.interval).end ==
                       std::numeric_limits<int>::max();
        }

        void get_sipp_neighbors(const SIPPState& s, std::vector<Neighbor<SIPPState, SIPPAction> >& neighbors)
        {
            std::vector<Neighbor<Location, Action> > motions;
            m_env.get_neighbors(s.state, motions);
            for (const auto& m : motions)
            {
                // std::cout << "gN " << m.state << std::endl;
                int m_time = m.cost;
                // std::cout << m_lastGScore;
                int start_t = m_lastGScore + m_time;
                int end_t = safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

                const auto& sis = safeIntervals(m_env.getLocation(m.location));
                for (size_t i = 0; i < sis.size(); ++i)
                {
                    const Interval& si = sis[i];
                    // std::cout << "  i " << i << ": " << si.start << "," << si.end <<
                    // std::endl;
                    if (si.start - m_time > end_t || si.end < start_t)
                    {
                        continue;
                    }

                    int t;
                    if (m_env.isCommandValid(s.state, m.location, m.action, m_lastGScore,
                                             end_t, si.start, si.end, t)) {
                        // std::cout << "  gN: " << m.state << "," << i << "," << t << ","
                        // << m_lastGScore << std::endl;
                        neighbors.emplace_back(Neighbor<SIPPState, SIPPAction>(
                            SIPPState(m.location, i), SIPPAction(m.action, m.cost), t - m_lastGScore));
                    }
                }
            }
        }

        void onExpandNode(const SIPPState& s, int fScore, int gScore)
        {
            // const auto& interval =
            // safeIntervals(m_env.getLocation(s.state)).at(s.interval);
            // std::cout << "expand: " << s.state << "," << interval.start << " to "
            // << interval.end << "(g: " << gScore << " f: " << fScore << ")" <<
            // std::endl;
            // This is called before get_neighbors(). We use the callback to find the
            // current cost (=time) of the expanded node
            m_lastGScore = gScore;
            m_env.onExpandNode(s.state, fScore, gScore);
        }

        void onDiscover(const SIPPState& s, int fScore, int gScore)
        {
            // const auto& interval =
            // safeIntervals(m_env.getLocation(s.state)).at(s.interval);
            // std::cout << "discover: " << s.state << "," << interval.start << " to "
            // << interval.end << std::endl;
            m_env.onDiscover(s.state, fScore, gScore);
        }

        void setCollisionIntervals(const Location& location, const std::vector<Interval>& intervals)
        {
            m_safeIntervals.erase(location);
            std::vector<Interval> sortedIntervals(intervals);
            std::sort(sortedIntervals.begin(), sortedIntervals.end());

            // std::cout << location << ": " << std::endl;
            if (intervals.size() > 0)
            {
                m_safeIntervals[location]; // create empty safe interval
                int start = 0;
                int lastEnd = 0;
                for (const auto& interval : sortedIntervals)
                {
                    // std::cout << "  ci: " << interval.start << " - " << interval.end <<
                    // std::endl;
                    assert(interval.start <= interval.end);
                    assert(start <= interval.start);
                    // if (start + 1 != interval.start - 1) {
                    // std::cout << start << "," << interval.start << std::endl;
                    // assert(start + 1 < interval.start - 1);
                    if (start <= interval.start - 1)
                    {
                        m_safeIntervals[location].push_back({start, interval.start - 1});
                    }
                    // }
                    start = interval.end + 1;
                    lastEnd = interval.end;
                }
                if (lastEnd < std::numeric_limits<int>::max())
                {
                    // assert(start < std::numeric_limits<int>::max());
                    m_safeIntervals[location].push_back({start, std::numeric_limits<int>::max()});
                }
            }

            // auto iter = m_safeIntervals.find(location);
            // if (iter != m_safeIntervals.end()) {
            //   for (const auto& si : iter->second) {
            //     std::cout << "  si: " << si.start << " - " << si.end << std::endl;
            //   }
            // }
        }

        bool findSafeInterval(const Location& state, int time, size_t& interval)
        {
            const auto& si = safeIntervals(m_env.getLocation(state));
            for (size_t idx = 0; idx < si.size(); ++idx)
            {
                if (si[idx].start <= time && si[idx].end >= time)
                {
                    interval = idx;

                    return true;
                }
            }

            return false;
        }

    private:
        const std::vector<Interval>& safeIntervals(const Location& location)
        {
            static std::vector<Interval> defaultInterval(1, {0, std::numeric_limits<int>::max()});
            const auto iter = m_safeIntervals.find(location);

            if (iter == m_safeIntervals.end())
            {
                return defaultInterval;
            }

            return iter->second;
        }

    private:
        Environment& m_env;
        int m_lastGScore;
        std::unordered_map<Location, std::vector<Interval> > m_safeIntervals;
    };

private:
    SIPPEnvironment m_env;
    AStar<SIPPEnvironment, SIPPStateHasher> m_astar;
};

#endif  // SIPP_ISOLATED_HPP
