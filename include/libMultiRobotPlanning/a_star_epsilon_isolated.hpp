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
};

class AStarEpsilonNode
{
public:
    using openSet_t = typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;
    using fibHeapHandle_t = typename openSet_t::handle_type;

    Location state;

    int f_score;
    int g_score;
    int focal_heuristic;

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
              g_score(input_gScore),
              focal_heuristic(input_focalHeuristic)
    {}

    bool operator<(const AStarEpsilonNode& other) const
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

    friend std::ostream& operator<<(std::ostream& os, const AStarEpsilonNode& node)
    {
        os << "state: " << node.state << " f_score: " << node.f_score
           << " g_score: " << node.g_score << " focal: " << node.focal_heuristic;

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
        // 1. lowest focal_heuristic
        // 2. lowest f_score
        // 3. highest g_score

        // Our heap is a maximum heap, so we invert the comperator function here
        if ((*h1).focal_heuristic != (*h2).focal_heuristic)
        {
            return (*h1).focal_heuristic > (*h2).focal_heuristic;
            // } else if ((*h1).f_score != (*h2).f_score) {
            //   return (*h1).f_score > (*h2).f_score;
        }
        else if ((*h1).f_score != (*h2).f_score)
        {
            return (*h1).f_score > (*h2).f_score;
        }
        else
        {
            return (*h1).g_score < (*h2).g_score;
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

    // debug var;
    size_t num_expanded_nodes;
    size_t num_generated_nodes;

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
              factor_w(input_w),
              num_expanded_nodes(0),
              num_generated_nodes(0)
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

    void get_neighbors(const Location& s, std::vector<Child>& children)
    {
        children.clear();

        Location up(s.x, s.y + 1);
        if (location_valid(up))
        {
            children.emplace_back(Child(up, Action::Up, 1));
        }

        Location down(s.x, s.y - 1);
        if (location_valid(down))
        {
            children.emplace_back(Child(down, Action::Down, 1));
        }

        Location left(s.x - 1, s.y);
        if (location_valid(left))
        {
            children.emplace_back(Child(left, Action::Left, 1));
        }

        Location right(s.x + 1, s.y);
        if (location_valid(right))
        {
            children.emplace_back(Child(right, Action::Right, 1));
        }
    }

    bool a_star_epsilon_search(const Location& startState, AgentPlan& solution)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(startState, 0));
        solution.actions.clear();
        solution.cost = 0;

        openSet_t open_set;
        focalSet_t focal_set;  // subset of open nodes that are within suboptimality bound
        std::unordered_map<Location, fibHeapHandle_t, std::hash<Location>> location_to_heaphandle;
        std::unordered_set<Location, std::hash<Location>> closed_set;
        std::unordered_map<Location, std::tuple<Location, Action, int, int>, std::hash<Location>> came_from;

        auto handle = open_set.push(AStarEpsilonNode(startState, admissible_heuristic(startState), 0, 0));
        location_to_heaphandle.insert(std::make_pair<>(startState, handle));
        (*handle).handle = handle;

        focal_set.push(handle);

        std::vector<Child> children;
        children.reserve(10);

        int best_f_score = (*handle).f_score;

        // std::cout << "new search" << std::endl;

        size_t num_iters = 0;
        while (!open_set.empty())
        {
            // update focal list
            int oldBestFScore = best_f_score;
            best_f_score = open_set.top().f_score;
            // std::cout << "best_f_score: " << best_f_score << std::endl;
            if (best_f_score > oldBestFScore)
            {
                // std::cout << "oldBestFScore: " << oldBestFScore << " newBestFScore:
                // " << best_f_score << std::endl;
                auto iter = open_set.ordered_begin();
                auto iterEnd = open_set.ordered_end();
                for (; iter != iterEnd; ++iter)
                {
                    int val = iter->f_score;
                    if (val > oldBestFScore * factor_w && val <= best_f_score * factor_w)
                    {
                        const AStarEpsilonNode& n = *iter;
                        focal_set.push(n.handle);
                    }
                    if (val > best_f_score * factor_w)
                    {
                        break;
                    }
                }
            }

            auto currentHandle = focal_set.top();
            AStarEpsilonNode current = *currentHandle;
            num_expanded_nodes++;

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
                solution.cost = current.g_score;

                std::cerr << "num expanded nodes: " << num_expanded_nodes << std::endl;
                std::cerr << "num generated nodes: " << num_generated_nodes << std::endl;
                std::cerr << "num iters: " << num_iters << std::endl;

                return true;
            }

            focal_set.pop();
            open_set.erase(currentHandle);
            location_to_heaphandle.erase(current.state);
            closed_set.insert(current.state);

            // traverse children
            children.clear();
            get_neighbors(current.state, children);

            for (const Child& neighbor : children)
            {
                if (closed_set.find(neighbor.location) == closed_set.end())
                {
                    int tentative_gScore = current.g_score + neighbor.cost;
                    auto iter = location_to_heaphandle.find(neighbor.location);
                    if (iter == location_to_heaphandle.end())
                    {  // Discover a new node
                        // std::cout << "  this is a new node" << std::endl;
                        int f_score = tentative_gScore + admissible_heuristic(neighbor.location);
                        int focal_heuristic = current.focal_heuristic + tentative_gScore +
                                             - current.g_score + tentative_gScore;
                        auto handle = open_set.push(
                                AStarEpsilonNode(neighbor.location, f_score, tentative_gScore, focal_heuristic));
                        (*handle).handle = handle;
                        if (f_score <= best_f_score * factor_w)
                        {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focal_set.push(handle);
                        }

                        location_to_heaphandle.insert(std::make_pair<>(neighbor.location, handle));
                        num_generated_nodes++;
                        // std::cout << "  this is a new node " << f_score << "," <<
                        // tentative_gScore << std::endl;
                    }
                    else
                    {
                        auto handle = iter->second;
                        // We found this node before with a better path
                        if (tentative_gScore >= (*handle).g_score)
                        {
                            continue;
                        }

                        int last_gScore = (*handle).g_score;
                        int last_fScore = (*handle).f_score;
                        // std::cout << "  this is an old node: " << tentative_gScore << ","
                        // << last_gScore << " " << *handle << std::endl;
                        // update f and g_score
                        int delta = last_gScore - tentative_gScore;
                        (*handle).g_score = tentative_gScore;
                        (*handle).f_score -= delta;
                        open_set.increase(handle);
                        num_generated_nodes++;

                        if ((*handle).f_score <= best_f_score * factor_w && last_fScore > best_f_score * factor_w)
                        {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focal_set.push(handle);
                        }
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "came_from", but otherwise
                    // default c'tors of Location and Action are required
                    came_from.erase(neighbor.location);
                    came_from.insert(std::make_pair<>(neighbor.location,
                     std::make_tuple<>(current.state, neighbor.action, neighbor.cost, tentative_gScore)));
                }
            }

            num_iters++;
        }

        return false;
    }
};

#endif //A_STAR_EPSILON_ISOLATED_HPP
