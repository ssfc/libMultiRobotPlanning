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
\tparam AStarEpsilon This class needs to provide the custom logic. In
    particular, it needs to support the following functions:
  - `Cost admissible_heuristic(const Location& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `bool is_solution(const Location& s)`\n
    Return true if the given state is a goal state.

  - `void get_neighbors(const Location& s, std::vector<Child<Location, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring state for the given state s.

  - `void onExpandNode(const Location& s, int f_score, int gScore)`\n
    This function is called on every expansion and can be used for statistical
purposes.

  - `void onDiscover(const Location& s, int f_score, int gScore)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

    \tparam LocationHasher A class to convert a state to a hash value. Default:
   std::hash<Location>
*/
class AStarEpsilonNode
{
public:
    using openSet_t = typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;
    using fibHeapHandle_t = typename openSet_t::handle_type;

    Location state;

    int f_score;
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
    AStarEpsilonNode(const Location& input_state, int input_f_score,
                     int input_gScore, int input_focalHeuristic)
            : state(input_state),
              f_score(input_f_score),
              gScore(input_gScore),
              focalHeuristic(input_focalHeuristic)
    {}

    bool operator<(const AStarEpsilonNode& other) const
    {
        // Sort order
        // 1. lowest f_score
        // 2. highest gScore

        // Our heap is a maximum heap, so we invert the comperator function here
        if (f_score != other.f_score)
        {
            return f_score > other.f_score;
        }
        else
        {
            return gScore < other.gScore;
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const AStarEpsilonNode& node)
    {
        os << "state: " << node.state << " f_score: " << node.f_score
           << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;

        return os;
    }
};


struct compareFocalHeuristic
{
    using openSet_t = typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;
    using fibHeapHandle_t = typename openSet_t::handle_type;

    bool operator()(const fibHeapHandle_t& h1, const fibHeapHandle_t& h2) const
    {
        // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
        // Path Finding" by Cohen et. al.)
        // 1. lowest focalHeuristic
        // 2. lowest f_score
        // 3. highest gScore

        // Our heap is a maximum heap, so we invert the comperator function here
        if ((*h1).focalHeuristic != (*h2).focalHeuristic)
        {
            return (*h1).focalHeuristic > (*h2).focalHeuristic;
            // } else if ((*h1).f_score != (*h2).f_score) {
            //   return (*h1).f_score > (*h2).f_score;
        }
        else if ((*h1).f_score != (*h2).f_score)
        {
            return (*h1).f_score > (*h2).f_score;
        }
        else
        {
            return (*h1).gScore < (*h2).gScore;
        }
    }
};


class AStarEpsilon
{
private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location goal;

    float factor_w;

    using openSet_t = typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;
    using fibHeapHandle_t = typename openSet_t::handle_type;
    using focalSet_t = typename boost::heap::d_ary_heap<fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compareFocalHeuristic> >;

public:
    AStarEpsilon(size_t dimx, size_t dimy, std::unordered_set<Location> input_obstacles,
                Location input_goal, float input_w)
            : num_columns(dimx),
              num_rows(dimy),
              obstacles(std::move(input_obstacles)),
              goal(std::move(input_goal)),  // NOLINT
              factor_w(input_w)
    {
        // std::cerr << "factor_w: " << factor_w << std::endl;
    }

    bool location_valid(const Location& s)
    {
        return s.x >= 0 && s.x < num_columns &&
            s.y >= 0 && s.y < num_rows &&
            obstacles.find(s) == obstacles.end();
    }

    int admissible_heuristic(const Location& s)
    {
        return std::abs(s.x - goal.x) + std::abs(s.y - goal.y);
    }

    bool is_solution(const Location& s)
    {
        return s == goal;
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

    void onExpandNode(const Location& /*s*/, int /*f_score*/, int /*gScore*/) {}

    void onDiscover(const Location& /*s*/, int /*f_score*/, int /*gScore*/) {}

    bool a_star_epsilon_search(const Location& startState, AgentPlan& solution)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(startState, 0));
        solution.actions.clear();
        solution.cost = 0;

        openSet_t openSet;
        focalSet_t focalSet;  // subset of open nodes that are within suboptimality bound
        std::unordered_map<Location, fibHeapHandle_t, std::hash<Location>> stateToHeap;
        std::unordered_set<Location, std::hash<Location>> closedSet;
        std::unordered_map<Location, std::tuple<Location, Action, int, int>, std::hash<Location>> cameFrom;

        auto handle = openSet.push(AStarEpsilonNode(startState, admissible_heuristic(startState), 0, 0));
        stateToHeap.insert(std::make_pair<>(startState, handle));
        (*handle).handle = handle;

        focalSet.push(handle);

        std::vector<Child> neighbors;
        neighbors.reserve(10);

        int bestFScore = (*handle).f_score;

        // std::cout << "new search" << std::endl;

        while (!openSet.empty())
        {
            // update focal list
            int oldBestFScore = bestFScore;
            bestFScore = openSet.top().f_score;
            // std::cout << "bestFScore: " << bestFScore << std::endl;
            if (bestFScore > oldBestFScore)
            {
                // std::cout << "oldBestFScore: " << oldBestFScore << " newBestFScore:
                // " << bestFScore << std::endl;
                auto iter = openSet.ordered_begin();
                auto iterEnd = openSet.ordered_end();
                for (; iter != iterEnd; ++iter)
                {
                    int val = iter->f_score;
                    if (val > oldBestFScore * factor_w && val <= bestFScore * factor_w)
                    {
                        const AStarEpsilonNode& n = *iter;
                        focalSet.push(n.handle);
                    }
                    if (val > bestFScore * factor_w)
                    {
                        break;
                    }
                }
            }

            auto currentHandle = focalSet.top();
            AStarEpsilonNode current = *currentHandle;
            onExpandNode(current.state, current.f_score, current.gScore);

            if (is_solution(current.state))
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
                solution.fmin = openSet.top().f_score;

                return true;
            }

            focalSet.pop();
            openSet.erase(currentHandle);
            stateToHeap.erase(current.state);
            closedSet.insert(current.state);

            // traverse neighbors
            neighbors.clear();
            get_neighbors(current.state, neighbors);

            for (const Child& neighbor : neighbors)
            {
                if (closedSet.find(neighbor.location) == closedSet.end())
                {
                    int tentative_gScore = current.gScore + neighbor.cost;
                    auto iter = stateToHeap.find(neighbor.location);
                    if (iter == stateToHeap.end())
                    {  // Discover a new node
                        // std::cout << "  this is a new node" << std::endl;
                        int f_score = tentative_gScore + admissible_heuristic(neighbor.location);
                        int focalHeuristic = current.focalHeuristic + tentative_gScore +
                                             - current.gScore + tentative_gScore;
                        auto handle = openSet.push(
                                AStarEpsilonNode(neighbor.location, f_score, tentative_gScore, focalHeuristic));
                        (*handle).handle = handle;
                        if (f_score <= bestFScore * factor_w)
                        {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focalSet.push(handle);
                        }

                        stateToHeap.insert(std::make_pair<>(neighbor.location, handle));
                        onDiscover(neighbor.location, f_score, tentative_gScore);
                        // std::cout << "  this is a new node " << f_score << "," <<
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

                        int last_gScore = (*handle).gScore;
                        int last_fScore = (*handle).f_score;
                        // std::cout << "  this is an old node: " << tentative_gScore << ","
                        // << last_gScore << " " << *handle << std::endl;
                        // update f and gScore
                        int delta = last_gScore - tentative_gScore;
                        (*handle).gScore = tentative_gScore;
                        (*handle).f_score -= delta;
                        openSet.increase(handle);
                        onDiscover(neighbor.location, (*handle).f_score, (*handle).gScore);

                        if ((*handle).f_score <= bestFScore * factor_w && last_fScore > bestFScore * factor_w)
                        {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focalSet.push(handle);
                        }
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "cameFrom", but otherwise
                    // default c'tors of Location and Action are required
                    cameFrom.erase(neighbor.location);
                    cameFrom.insert(std::make_pair<>(neighbor.location,
                                                     std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                                                       tentative_gScore)));
                }
            }

        }

        return false;
    }
};

#endif //A_STAR_EPSILON_ISOLATED_HPP
