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

class Child
{
public:
    //! neighboring location
    Location location;
    //! action to get to the neighboring location
    Action action;
    //! cost to get to the neighboring location, usually 1
    int cost;

public:
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

    Location location;

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
    AStarEpsilonNode(const Location& input_location, int input_f_score,
                     int input_g_score, int input_focal_heuristic)
            : location(input_location),
              f_score(input_f_score),
              g_score(input_g_score),
              focal_heuristic(input_focal_heuristic)
    {}

    bool operator<(const AStarEpsilonNode& other) const
    {
        // Sort order
        // 1. lowest f_score
        // 2. highest g_score

        if (f_score != other.f_score)
        {
            return f_score > other.f_score; // 反向为小
        }
        else
        {
            return g_score < other.g_score; // 同向为大
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const AStarEpsilonNode& node)
    {
        os << "location: " << node.location << " f_score: " << node.f_score
           << " g_score: " << node.g_score << " focal: " << node.focal_heuristic;

        return os;
    }
};


struct compare_focal_heuristic
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

        // 1. lowest focal_heuristic
        if ((*h1).focal_heuristic != (*h2).focal_heuristic)
        {
            return (*h1).focal_heuristic > (*h2).focal_heuristic;
            // } else if ((*h1).f_score != (*h2).f_score) {
            //   return (*h1).f_score > (*h2).f_score;
        }
        // 2. lowest f_score
        else if ((*h1).f_score != (*h2).f_score)
        {
            return (*h1).f_score > (*h2).f_score;
        }
        // 3. highest g_score
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
    Location start;
    Location goal;

    float factor_w;

    // debug var;
    size_t num_expanded_nodes;
    size_t num_generated_nodes;

    using openSet_t = typename boost::heap::d_ary_heap<AStarEpsilonNode, boost::heap::arity<2>, boost::heap::mutable_<true> >;

    using fibHeapHandle_t = typename openSet_t::handle_type;
    using focalSet_t = typename boost::heap::d_ary_heap<fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compare_focal_heuristic> >;

public:
    AStarEpsilon(size_t dimx, size_t dimy, std::unordered_set<Location> input_obstacles,
                Location input_start, Location input_goal, float input_w)
            : num_columns(dimx),
              num_rows(dimy),
              obstacles(std::move(input_obstacles)),
              start(std::move(input_start)),
              goal(std::move(input_goal)),  // NOLINT
              factor_w(input_w),
              num_expanded_nodes(0),
              num_generated_nodes(0)
    {
        // std::cerr << "factor_w: " << factor_w << std::endl;
    }

    bool location_valid(const Location& input_location)
    {
        return input_location.x >= 0 && input_location.x < num_columns &&
            input_location.y >= 0 && input_location.y < num_rows &&
            obstacles.find(input_location) == obstacles.end();
    }

    int calculate_h(const Location& input_location)
    {
        return std::abs(input_location.x - goal.x) + std::abs(input_location.y - goal.y);
    }

    bool is_solution(const Location& input_location)
    {
        return input_location == goal;
    }

    void get_neighbors(const Location& input_location, std::vector<Child>& children)
    {
        children.clear();

        Location up(input_location.x, input_location.y + 1);
        if (location_valid(up))
        {
            children.emplace_back(Child(up, Action::Up, 1));
        }

        Location down(input_location.x, input_location.y - 1);
        if (location_valid(down))
        {
            children.emplace_back(Child(down, Action::Down, 1));
        }

        Location left(input_location.x - 1, input_location.y);
        if (location_valid(left))
        {
            children.emplace_back(Child(left, Action::Left, 1));
        }

        Location right(input_location.x + 1, input_location.y);
        if (location_valid(right))
        {
            children.emplace_back(Child(right, Action::Right, 1));
        }
    }

    // A* LINE 1
    // A* finds a path from start to goal.
    // h is the heuristic function. h(n) estimates the cost to reach goal from node n.
    // function A_Star(start, goal, h_score)
    bool a_star_epsilon_search(AgentPlan& solution)
    {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(start, 0));
        solution.actions.clear();
        solution.cost = 0;

        // 1. lowest f_score // 2. highest g_score
        openSet_t open_set;

        // 1. lowest focal_heuristic // 2. lowest f_score // 3. highest g_score
        focalSet_t focal_set;  // subset of open nodes that are within suboptimality bound
        std::unordered_map<Location, fibHeapHandle_t, std::hash<Location>> location_to_heaphandle;
        std::unordered_set<Location, std::hash<Location>> closed_set;

        // A* LINE 2
        // For node n, came_from[n] is the node immediately preceding it on the cheapest path from the start
        // to n currently known.
        // came_from := an empty map
        std::unordered_map<Location, std::tuple<Location, Action, int, int>, std::hash<Location>> came_from;

        // A* LINE 3
        // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
        // gScore := map with default value of Infinity

        // A* LINE 4
        // gScore[start] := 0

        // A* LINE 5
        // For node n, fScore[n] := gScore[n] + h_score(n). fScore[n] represents our current best guess as to
        // how cheap a path could be from start to finish if it goes through n.
        // fScore := map with default value of Infinity

        // A* LINE 6
        // fScore[start] := h_score(start)
        // Initialising the parameters of the starting node

        // A* LINE 7
        // The set of discovered nodes that may need to be (re-)expanded.
        // Initially, only the start node is known.
        // This is usually implemented as a min-heap or priority queue rather than a hash-set.
        // open_set := {start}
        auto root_handle = open_set.emplace(AStarEpsilonNode(start, calculate_h(start), 0, 0));
        location_to_heaphandle.insert(std::make_pair<>(start, root_handle));
        (*root_handle).handle = root_handle;

        focal_set.emplace(root_handle); // focal set同步open set更新

        std::vector<Child> children;
        children.reserve(10);

        int best_f_score = (*root_handle).f_score;

        // std::cout << "new search" << std::endl;

        // A* LINE 8
        // while openSet is not empty
        size_t num_iters = 0;
        while (!open_set.empty())
        {
            // update focal list
            int old_best_f_score = best_f_score;
            best_f_score = open_set.top().f_score;
            // std::cout << "best_f_score: " << best_f_score << std::endl;
            if (best_f_score > old_best_f_score) // 还不如原来的。???
            {
                // std::cout << "old_best_f_score: " << old_best_f_score << " newBestFScore:
                // " << best_f_score << std::endl;
                auto iter = open_set.ordered_begin();
                auto iterEnd = open_set.ordered_end();
                // 遍历open_set
                for (; iter != iterEnd; ++iter)
                {
                    int val = iter->f_score;

                    // val <= old_best_f_score * factor_w已经在里面了，所以无需更新
                    if (val > old_best_f_score * factor_w && val <= best_f_score * factor_w)
                    {
                        const AStarEpsilonNode& node = *iter;
                        focal_set.emplace(node.handle);
                    }

                    if (val > best_f_score * factor_w) // 因为open_set是有序集，所以中断就可以了。
                    {
                        break;
                    }
                }
            }

            // A* LINE 9
            // This operation can occur in O(Log(N)) time if open_set is a min-heap or a priority queue
            // current := the node in open_set having the lowest f_score[] value
            // 弹出来的不是open set的top, 而是focal set的top
            // => 也可以理解，如果弹出的是open set的top, 不就是精确算法了？
            auto current_handle = focal_set.top();
            AStarEpsilonNode current = *current_handle;
            num_expanded_nodes++;

            // A* LINE 10
            // if current = goal
            if (is_solution(current.location))
            {
                solution.path.clear();
                solution.actions.clear();
                // A* LINE 11
                // return reconstruct_path(cameFrom, current)
                auto iter = came_from.find(current.location);
                while (iter != came_from.end())
                {
                    solution.path.emplace_back(
                            std::make_pair<>(iter->first, std::get<3>(iter->second)));
                    solution.actions.emplace_back(std::make_pair<>(
                            std::get<1>(iter->second), std::get<2>(iter->second)));
                    iter = came_from.find(std::get<0>(iter->second));
                }

                solution.path.emplace_back(std::make_pair<>(start, 0));
                std::reverse(solution.path.begin(), solution.path.end());
                std::reverse(solution.actions.begin(), solution.actions.end());
                solution.cost = current.g_score;

                std::cerr << "num expanded nodes: " << num_expanded_nodes << std::endl;
                std::cerr << "num generated nodes: " << num_generated_nodes << std::endl;
                std::cerr << "num iters: " << num_iters << std::endl;

                return true;
            }

            // A* LINE 12
            // openSet.Remove(current)
            // open_set和focal_set同步更新
            focal_set.pop();
            open_set.erase(current_handle);
            location_to_heaphandle.erase(current.location);
            closed_set.insert(current.location);

            // A* LINE 13
            // for each neighbor of current
            // traverse children
            children.clear();
            get_neighbors(current.location, children);

            for (const Child& neighbor : children)
            {
                // If the successor has not been evaluated
                if (closed_set.find(neighbor.location) == closed_set.end())
                {
                    // A* LINE 14
                    // d(current,neighbor) is the weight of the edge from current to neighbor
                    // tentative_g_score is the distance from start to the neighbor through current
                    // tentative_g_score := gScore[current] + d(current, neighbor)
                    // cost of the cheapest path from start to n currently known
                    int tentative_g_score = current.g_score + neighbor.cost;
                    auto iter = location_to_heaphandle.find(neighbor.location);
                    if (iter == location_to_heaphandle.end()) // 坐标不在堆中就不比较直接加（毕竟原g值是inf, 新g值肯定更小）
                    {  // Discover a new node
                        // std::cout << "  this is a new node" << std::endl;
                        int f_score = tentative_g_score + calculate_h(neighbor.location);
                        // focal_heuristic只在插入新节点时更新？
                        int focal_heuristic = current.focal_heuristic + tentative_g_score + neighbor.cost; // ???
                        auto child_handle = open_set.emplace(AStarEpsilonNode(neighbor.location,
                             f_score, tentative_g_score, focal_heuristic));
                        (*child_handle).handle = child_handle;
                        if (f_score <= best_f_score * factor_w)
                        {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focal_set.emplace(child_handle);
                        }

                        location_to_heaphandle.insert(std::make_pair<>(neighbor.location, child_handle));
                        num_generated_nodes++;
                        // std::cout << "  this is a new node " << f_score << "," <<
                        // tentative_g_score << std::endl;
                    }
                    else
                    {
                        auto child_handle = iter->second;
                        // We found this node before with a better path
                        if (tentative_g_score >= (*child_handle).g_score)
                        {
                            continue;
                        }

                        int last_fScore = (*child_handle).f_score;
                        // std::cout << "  this is an old node: " << tentative_g_score << ","
                        // << last_gScore << " " << *child_handle << std::endl;
                        // update f and g_score
                        (*child_handle).g_score = tentative_g_score;
                        (*child_handle).f_score = tentative_g_score + calculate_h(neighbor.location);
                        open_set.increase(child_handle); // 这里不需要更新focal_set
                        num_generated_nodes++;

                        if ((*child_handle).f_score <= best_f_score * factor_w && last_fScore > best_f_score * factor_w)
                        {
                            // std::cout << "focalAdd: " << *child_handle << std::endl;
                            focal_set.emplace(child_handle);
                        }
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "came_from", but otherwise
                    // default c'tors of Location and Action are required
                    came_from.erase(neighbor.location);
                    came_from.insert(std::make_pair<>(neighbor.location,
                     std::make_tuple<>(current.location, neighbor.action, neighbor.cost, tentative_g_score)));
                }
            }

            num_iters++;
        }

        return false;
    }
};

#endif //A_STAR_EPSILON_ISOLATED_HPP
