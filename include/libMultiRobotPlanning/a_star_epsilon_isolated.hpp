//
// Created by take_ on 2023/11/25.
//

#ifndef A_STAR_EPSILON_ISOLATED_HPP
#define A_STAR_EPSILON_ISOLATED_HPP

#pragma once

#include <boost/heap/fibonacci_heap.hpp>

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "util.hpp"
#include "neighbor.hpp"
#include "planresult.hpp"

// #define REBUILT_FOCAL_LIST
// #define CHECK_FOCAL_LIST

enum class Action
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
    //! lower bound of the cost (for suboptimal solvers)
    int fmin;
};

class Environment
{
private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location m_goal;
public:
    Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
                Location goal)
            : num_columns(dimx),
              num_rows(dimy),
              obstacles(std::move(obstacles)),
              m_goal(std::move(goal))  // NOLINT
    {}

    bool location_valid(const Location& s)
    {
        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
               obstacles.find(s) == obstacles.end();
    }

    int admissible_heuristic(const Location& s)
    {
        return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
    }

    // a potentially inadmissible heuristic
    int focalStateHeuristic(const Location& /*s*/, int gScore)
    {
        // prefer lower g-values
        return gScore;
    }

    int focalTransitionHeuristic(const Location& /*s1*/, const Location& /*s2*/,
                                 int gScoreS1, int gScoreS2)
    {
        // prefer lower g-values
        return gScoreS2 - gScoreS1;
    }

    bool is_solution(const Location& s)
    {
        return s == m_goal;
    }

    void get_neighbors(const Location& s, std::vector<Child>& neighbors)
    {
        neighbors.clear();

        Location up(s.x, s.y + 1);
        if (location_valid(up))
        {
            neighbors.emplace_back(Child(up, Action::Up, 1));
        }

        Location down(s.x, s.y - 1);
        if (location_valid(down))
        {
            neighbors.emplace_back(Child(down, Action::Down, 1));
        }

        Location left(s.x - 1, s.y);
        if (location_valid(left))
        {
            neighbors.emplace_back(Child(left, Action::Left, 1));
        }

        Location right(s.x + 1, s.y);
        if (location_valid(right))
        {
            neighbors.emplace_back(Child(right, Action::Right, 1));
        }
    }

    void onExpandNode(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}

    void onDiscover(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}
};

/*!
  \example a_star_epsilon.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief A*_epsilon Algorithm to find the shortest path with a given
suboptimality bound (also known as focal search)

This class implements the A*_epsilon algorithm, an informed search
algorithm
that finds the shortest path for a given map up to a suboptimality factor.
It uses an admissible heuristic (to keep track of the optimum) and an
inadmissible heuristic (
to guide the search within a suboptimal bound w.)

Details of the algorithm can be found in the following paper:\n
Judea Pearl, Jin H. Kim:\n
"Studies in Semi-Admissible Heuristics."" IEEE Trans. Pattern Anal. Mach.
Intell. 4(4): 392-399 (1982)\n
https://doi.org/10.1109/TPAMI.1982.4767270

This class can either use a fibonacci heap, or a d-ary heap. The latter is the
default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap instead.

\tparam Location Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom logic. In
    particular, it needs to support the following functions:
  - `Cost admissible_heuristic(const Location& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `Cost focalStateHeuristic(const Location& s, Cost gScore)`\n
    This function computes a (potentially inadmissible) heuristic for the given
state.

  - `Cost focalTransitionHeuristic(const Location& s1, const Location& s2, Cost
gScoreS1, Cost gScoreS2)`\n
    This function computes a (potentially inadmissible) heuristic for the given
state transition.

  - `bool is_solution(const Location& s)`\n
    Return true if the given state is a goal state.

  - `void get_neighbors(const Location& s, std::vector<Child<Location, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring state for the given state s.

  - `void onExpandNode(const Location& s, int fScore, int gScore)`\n
    This function is called on every expansion and can be used for statistical
purposes.

  - `void onDiscover(const Location& s, int fScore, int gScore)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

    \tparam LocationHasher A class to convert a state to a hash value. Default:
   std::hash<Location>
*/
class AStarEpsilonNode
{
public:
    typedef typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>, boost::heap::mutable_<true> > openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;

    Location state;

    int fScore;
    int gScore;
    int focalHeuristic;

    fibHeapHandle_t handle;
    // #ifdef USE_FIBONACCI_HEAP
    //   typename boost::heap::fibonacci_heap<AStarEpsilonNode>::handle_type handle;
    // #else
    //   typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>,
    //   boost::heap::mutable_<true> >::handle_type handle;
    // #endif

public:
    AStarEpsilonNode(const Location& state, int fScore, int gScore, int focalHeuristic)
            : state(state),
              fScore(fScore),
              gScore(gScore),
              focalHeuristic(focalHeuristic)
    {}

    bool operator<(const AStarEpsilonNode& other) const
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

    friend std::ostream& operator<<(std::ostream& os, const AStarEpsilonNode& node)
    {
        os << "state: " << node.state << " fScore: " << node.fScore
           << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;

        return os;
    }
};

class AStarEpsilon
{
private:
    Environment& m_env;
    float m_w;

public:
    AStarEpsilon(Environment& environment, float w)
            : m_env(environment), m_w(w)
            {}

    bool a_star_epsilon_search(const Location& startState, AgentPlan& solution)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(startState, 0));
        solution.actions.clear();
        solution.cost = 0;

        openSet_t openSet;
        focalSet_t
                focalSet;  // subset of open nodes that are within suboptimality bound
        std::unordered_map<Location, fibHeapHandle_t, std::hash<Location>> stateToHeap;
        std::unordered_set<Location, std::hash<Location>> closedSet;
        std::unordered_map<Location, std::tuple<Location, Action, int, int>, std::hash<Location>> cameFrom;

        auto handle = openSet.push(
                AStarEpsilonNode(startState, m_env.admissible_heuristic(startState), 0, 0));
        stateToHeap.insert(std::make_pair<>(startState, handle));
        (*handle).handle = handle;

        focalSet.push(handle);

        std::vector<Child> neighbors;
        neighbors.reserve(10);

        int bestFScore = (*handle).fScore;

        // std::cout << "new search" << std::endl;

        while (!openSet.empty())
        {
        // update focal list
        #ifdef REBUILT_FOCAL_LIST
            focalSet.clear();
            const auto& top = openSet.top();
            int bestVal = top.fScore;
            auto iter = openSet.ordered_begin();
            auto iterEnd = openSet.ordered_end();
            for (; iter != iterEnd; ++iter)
            {
                int val = iter->fScore;
                if (val <= bestVal * m_w)
                {
                    const auto& s = *iter;
                    focalSet.push(s.handle);
                }
                else
                {
                    break;
                }
            }
        #else

            int oldBestFScore = bestFScore;
            bestFScore = openSet.top().fScore;
            // std::cout << "bestFScore: " << bestFScore << std::endl;
            if (bestFScore > oldBestFScore)
            {
                // std::cout << "oldBestFScore: " << oldBestFScore << " newBestFScore:
                // " << bestFScore << std::endl;
                auto iter = openSet.ordered_begin();
                auto iterEnd = openSet.ordered_end();
                for (; iter != iterEnd; ++iter)
                {
                    int val = iter->fScore;
                    if (val > oldBestFScore * m_w && val <= bestFScore * m_w)
                    {
                        const AStarEpsilonNode& n = *iter;
                        focalSet.push(n.handle);
                    }
                    if (val > bestFScore * m_w)
                    {
                        break;
                    }
                }
            }

        #endif
        // check focal list/open list consistency
        #ifdef CHECK_FOCAL_LIST

            // focalSet_t focalSetGolden;
            bool mismatch = false;
            const auto& top = openSet.top();
            int bestVal = top.fScore;
            auto iter = openSet.ordered_begin();
            auto iterEnd = openSet.ordered_end();
            for (; iter != iterEnd; ++iter)
            {
                const auto& s = *iter;
                int val = s.fScore;
                if (val <= bestVal * m_w)
                {
                    // std::cout << "should: " << s << std::endl;
                    // focalSetGolden.push(s.handle);
                    if (std::find(focalSet.begin(), focalSet.end(), s.handle) == focalSet.end())
                    {
                        std::cout << "focalSet misses: " << s << std::endl;
                        mismatch = true;
                    }

                }
                else
                {
                    if (std::find(focalSet.begin(), focalSet.end(), s.handle) != focalSet.end())
                    {
                        std::cout << "focalSet shouldn't have: " << s << std::endl;
                        mismatch = true;
                    }
                    // break;
                }
            }

            assert(!mismatch);
            // assert(focalSet == focalSetGolden);

        #endif

            auto currentHandle = focalSet.top();
            AStarEpsilonNode current = *currentHandle;
            m_env.onExpandNode(current.state, current.fScore, current.gScore);

            if (m_env.is_solution(current.state))
            {
                solution.path.clear();
                solution.actions.clear();
                auto iter = cameFrom.find(current.state);
                while (iter != cameFrom.end())
                {
                    solution.path.emplace_back(
                            std::make_pair<>(iter->first, std::get<3>(iter->second)));
                    solution.actions.emplace_back(std::make_pair<>(
                            std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = cameFrom.find(std::get<0>(iter->second));
                }

                solution.path.emplace_back(std::make_pair<>(startState, 0));
                std::reverse(solution.path.begin(), solution.path.end());
                std::reverse(solution.actions.begin(), solution.actions.end());
                solution.cost = current.gScore;
                solution.fmin = openSet.top().fScore;

                return true;
            }

            focalSet.pop();
            openSet.erase(currentHandle);
            stateToHeap.erase(current.state);
            closedSet.insert(current.state);

            // traverse neighbors
            neighbors.clear();
            m_env.get_neighbors(current.state, neighbors);

            for (const Child& neighbor : neighbors)
            {
                if (closedSet.find(neighbor.location) == closedSet.end())
                {
                    int tentative_gScore = current.gScore + neighbor.cost;
                    auto iter = stateToHeap.find(neighbor.location);
                    if (iter == stateToHeap.end()) {  // Discover a new node
                        // std::cout << "  this is a new node" << std::endl;
                        int fScore =
                                tentative_gScore + m_env.admissible_heuristic(neighbor.location);
                        int focalHeuristic =
                                current.focalHeuristic +
                                m_env.focalStateHeuristic(neighbor.location, tentative_gScore) +
                                m_env.focalTransitionHeuristic(current.state, neighbor.location,
                                                               current.gScore,
                                                               tentative_gScore);
                        auto handle = openSet.push(
                                AStarEpsilonNode(neighbor.location, fScore, tentative_gScore, focalHeuristic));
                        (*handle).handle = handle;
                        if (fScore <= bestFScore * m_w) {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focalSet.push(handle);
                        }
                        stateToHeap.insert(std::make_pair<>(neighbor.location, handle));
                        m_env.onDiscover(neighbor.location, fScore, tentative_gScore);
                        // std::cout << "  this is a new node " << fScore << "," <<
                        // tentative_gScore << std::endl;
                    } else {
                        auto handle = iter->second;
                        // We found this node before with a better path
                        if (tentative_gScore >= (*handle).gScore) {
                            continue;
                        }
                        int last_gScore = (*handle).gScore;
                        int last_fScore = (*handle).fScore;
                        // std::cout << "  this is an old node: " << tentative_gScore << ","
                        // << last_gScore << " " << *handle << std::endl;
                        // update f and gScore
                        int delta = last_gScore - tentative_gScore;
                        (*handle).gScore = tentative_gScore;
                        (*handle).fScore -= delta;
                        openSet.increase(handle);
                        m_env.onDiscover(neighbor.location, (*handle).fScore,
                                         (*handle).gScore);
                        if ((*handle).fScore <= bestFScore * m_w &&
                            last_fScore > bestFScore * m_w) {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focalSet.push(handle);
                        }
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "cameFrom", but otherwise
                    // default c'tors of Location and Action are required
                    cameFrom.erase(neighbor.location);
                    cameFrom.insert(std::make_pair<>(
                            neighbor.location,
                            std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                              tentative_gScore)));
                }
            }

        }

        return false;
    }

private:

    typedef typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>, boost::heap::mutable_<true> > openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;
// typedef typename boost::heap::d_ary_heap<fibHeapHandle_t,
// boost::heap::arity<2>, boost::heap::mutable_<true>,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;



    struct compareFocalHeuristic
    {
        bool operator()(const fibHeapHandle_t& h1, const fibHeapHandle_t& h2) const
        {
            // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
            // Path Finding" by Cohen et. al.)
            // 1. lowest focalHeuristic
            // 2. lowest fScore
            // 3. highest gScore

            // Our heap is a maximum heap, so we invert the comperator function here
            if ((*h1).focalHeuristic != (*h2).focalHeuristic)
            {
                return (*h1).focalHeuristic > (*h2).focalHeuristic;
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

    // typedef typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>,
    // boost::heap::mutable_<true> > openSet_t;
    // typedef typename openSet_t::handle_type fibHeapHandle_t;
    typedef typename boost::heap::d_ary_heap<
            fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compareFocalHeuristic> > focalSet_t;
};

#endif //A_STAR_EPSILON_ISOLATED_HPP
