//
// Created by take_ on 2023/12/14.
//

#ifndef ECBS_ISOLATED_HPP
#define ECBS_ISOLATED_HPP

#include <map>

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "util.hpp"


///
enum class Action {
    Up,
    Down,
    Left,
    Right,
    Wait,
};

struct Neighbor
{
    //! neighboring location
    TimeLocation time_location;
    //! action to get to the neighboring location
    Action action;
    //! cost to get to the neighboring location, usually 1
    int cost;

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


template <typename LowLevelEnvironment>
class AStarEpsilon {
public:
    AStarEpsilon(LowLevelEnvironment& environment, float w)
            : m_env(environment), m_w(w) {}

    bool search(const TimeLocation& startState,
                PlanResult& solution) {
        solution.path.clear();
        solution.path.emplace_back(std::make_pair<>(startState, 0));
        solution.actions.clear();
        solution.cost = 0;

        openSet_t openSet;
        focalSet_t
                focalSet;  // subset of open nodes that are within suboptimality bound
        std::unordered_map<TimeLocation, fibHeapHandle_t, std::hash<TimeLocation>> stateToHeap;
        std::unordered_set<TimeLocation, std::hash<TimeLocation>> closedSet;
        std::unordered_map<TimeLocation, std::tuple<TimeLocation, Action, int, int>,
                std::hash<TimeLocation>> cameFrom;

        auto handle = openSet.push(
                Node(startState, m_env.admissible_heuristic(startState), 0, 0));
        stateToHeap.insert(std::make_pair<>(startState, handle));
        (*handle).handle = handle;

        focalSet.push(handle);

        std::vector<Neighbor> neighbors;
        neighbors.reserve(10);

        int bestFScore = (*handle).fScore;

        // std::cout << "new search" << std::endl;

        while (!openSet.empty()) {
// update focal list
#ifdef REBUILT_FOCAL_LIST
            focalSet.clear();
  const auto& top = openSet.top();
  int bestVal = top.fScore;
  auto iter = openSet.ordered_begin();
  auto iterEnd = openSet.ordered_end();
  for (; iter != iterEnd; ++iter) {
    int val = iter->fScore;
    if (val <= bestVal * m_w) {
      const auto& s = *iter;
      focalSet.push(s.handle);
    } else {
      break;
    }
  }
#else
            {
                int oldBestFScore = bestFScore;
                bestFScore = openSet.top().fScore;
                // std::cout << "bestFScore: " << bestFScore << std::endl;
                if (bestFScore > oldBestFScore) {
                    // std::cout << "oldBestFScore: " << oldBestFScore << " newBestFScore:
                    // " << bestFScore << std::endl;
                    auto iter = openSet.ordered_begin();
                    auto iterEnd = openSet.ordered_end();
                    for (; iter != iterEnd; ++iter) {
                        int val = iter->fScore;
                        if (val > oldBestFScore * m_w && val <= bestFScore * m_w) {
                            const Node& n = *iter;
                            focalSet.push(n.handle);
                        }
                        if (val > bestFScore * m_w) {
                            break;
                        }
                    }
                }
            }
#endif
// check focal list/open list consistency

            auto currentHandle = focalSet.top();
            Node current = *currentHandle;
            m_env.onExpandNode(current.state, current.fScore, current.gScore);

            if (m_env.is_solution(current.state)) {
                solution.path.clear();
                solution.actions.clear();
                auto iter = cameFrom.find(current.state);
                while (iter != cameFrom.end()) {
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
            for (const Neighbor& neighbor : neighbors) {
                if (closedSet.find(neighbor.time_location) == closedSet.end()) {
                    int tentative_gScore = current.gScore + neighbor.cost;
                    auto iter = stateToHeap.find(neighbor.time_location);
                    if (iter == stateToHeap.end()) {  // Discover a new node
                        // std::cout << "  this is a new node" << std::endl;
                        int fScore =
                                tentative_gScore + m_env.admissible_heuristic(neighbor.time_location);
                        int focalHeuristic =
                                current.focalHeuristic +
                                m_env.focalStateHeuristic(neighbor.time_location, tentative_gScore) +
                                m_env.focalTransitionHeuristic(current.state, neighbor.time_location,
                                                               current.gScore,
                                                               tentative_gScore);
                        auto handle = openSet.push(
                                Node(neighbor.time_location, fScore, tentative_gScore, focalHeuristic));
                        (*handle).handle = handle;
                        if (fScore <= bestFScore * m_w) {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focalSet.push(handle);
                        }
                        stateToHeap.insert(std::make_pair<>(neighbor.time_location, handle));
                        m_env.onDiscover(neighbor.time_location, fScore, tentative_gScore);
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
                        m_env.onDiscover(neighbor.time_location, (*handle).fScore,
                                         (*handle).gScore);
                        if ((*handle).fScore <= bestFScore * m_w &&
                            last_fScore > bestFScore * m_w) {
                            // std::cout << "focalAdd: " << *handle << std::endl;
                            focalSet.push(handle);
                        }
                    }

                    // Best path for this node so far
                    // TODO: this is not the best way to update "cameFrom", but otherwise
                    // default c'tors of TimeLocation and Action are required
                    cameFrom.erase(neighbor.time_location);
                    cameFrom.insert(std::make_pair<>(
                            neighbor.time_location,
                            std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                              tentative_gScore)));
                }
            }
        }

        return false;
    }

private:
    struct Node;

#ifdef USE_FIBONACCI_HEAP
    typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
typedef typename openSet_t::handle_type fibHeapHandle_t;
// typedef typename boost::heap::fibonacci_heap<fibHeapHandle_t,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#else
    typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
    boost::heap::mutable_<true> >
    openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;
// typedef typename boost::heap::d_ary_heap<fibHeapHandle_t,
// boost::heap::arity<2>, boost::heap::mutable_<true>,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#endif

    struct Node {
        Node(const TimeLocation& state, int fScore, int gScore, int focalHeuristic)
                : state(state),
                  fScore(fScore),
                  gScore(gScore),
                  focalHeuristic(focalHeuristic) {}

        bool operator<(const Node& other) const {
            // Sort order
            // 1. lowest fScore
            // 2. highest gScore

            // Our heap is a maximum heap, so we invert the comperator function here
            if (fScore != other.fScore) {
                return fScore > other.fScore;
            } else {
                return gScore < other.gScore;
            }
        }

        friend std::ostream& operator<<(std::ostream& os, const Node& node) {
            os << "state: " << node.state << " fScore: " << node.fScore
               << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;
            return os;
        }

        TimeLocation state;

        int fScore;
        int gScore;
        int focalHeuristic;

        fibHeapHandle_t handle;
        // #ifdef USE_FIBONACCI_HEAP
        //   typename boost::heap::fibonacci_heap<Node>::handle_type handle;
        // #else
        //   typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
        //   boost::heap::mutable_<true> >::handle_type handle;
        // #endif
    };

    struct compareFocalHeuristic {
        bool operator()(const fibHeapHandle_t& h1,
                        const fibHeapHandle_t& h2) const {
            // Sort order (see "Improved Solvers for Bounded-Suboptimal Multi-Agent
            // Path Finding" by Cohen et. al.)
            // 1. lowest focalHeuristic
            // 2. lowest fScore
            // 3. highest gScore

            // Our heap is a maximum heap, so we invert the comperator function here
            if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
                return (*h1).focalHeuristic > (*h2).focalHeuristic;
                // } else if ((*h1).fScore != (*h2).fScore) {
                //   return (*h1).fScore > (*h2).fScore;
            } else if ((*h1).fScore != (*h2).fScore) {
                return (*h1).fScore > (*h2).fScore;
            } else {
                return (*h1).gScore < (*h2).gScore;
            }
        }
    };

#ifdef USE_FIBONACCI_HEAP
    // typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
// typedef typename openSet_t::handle_type fibHeapHandle_t;
typedef typename boost::heap::fibonacci_heap<
  fibHeapHandle_t, boost::heap::compare<compareFocalHeuristic> >
  focalSet_t;
#else
    // typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
    // boost::heap::mutable_<true> > openSet_t;
    // typedef typename openSet_t::handle_type fibHeapHandle_t;
    typedef typename boost::heap::d_ary_heap<
            fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compareFocalHeuristic> >
    focalSet_t;
#endif

private:
    LowLevelEnvironment& m_env;
    float m_w;
};

template <typename Conflict,
        typename Constraints, typename Environment>
class ECBS {
public:
    ECBS(Environment& environment, float w) : m_env(environment), m_w(w) {}

    bool search(const std::vector<TimeLocation>& initialStates,
                std::vector<PlanResult>& solution) {
        HighLevelNode start;
        start.solution.resize(initialStates.size());
        start.constraints.resize(initialStates.size());
        start.cost = 0;
        start.LB = 0;
        start.id = 0;

        for (size_t i = 0; i < initialStates.size(); ++i) {
            if (i < solution.size() && solution[i].path.size() > 1) {
                std::cout << initialStates[i] << " " << solution[i].path.front().first
                          << std::endl;
                assert(initialStates[i] == solution[i].path.front().first);
                start.solution[i] = solution[i];
                std::cout << "use existing solution for agent: " << i << std::endl;
            } else {
                LowLevelEnvironment llenv(m_env, i, start.constraints[i],
                                          start.solution);
                LowLevelSearch_t lowLevel(llenv, m_w);
                bool success = lowLevel.search(initialStates[i], start.solution[i]);
                if (!success) {
                    return false;
                }
            }
            start.cost += start.solution[i].cost;
            start.LB += start.solution[i].fmin;
        }
        start.focalHeuristic = m_env.focalHeuristic(start.solution);

        // std::priority_queue<HighLevelNode> open;
        openSet_t open;
        focalSet_t focal;

        auto handle = open.push(start);
        (*handle).handle = handle;
        focal.push(handle);

        int bestCost = (*handle).cost;

        solution.clear();
        int id = 1;
        while (!open.empty()) {
// update focal list
#ifdef REBUILT_FOCAL_LIST
            focal.clear();
  int LB = open.top().LB;

  auto iter = open.ordered_begin();
  auto iterEnd = open.ordered_end();
  for (; iter != iterEnd; ++iter) {
    float val = iter->cost;
    // std::cout << "  cost: " << val << std::endl;
    if (val <= LB * m_w) {
      const HighLevelNode& node = *iter;
      focal.push(node.handle);
    } else {
      break;
    }
  }
#else
            {
                int oldBestCost = bestCost;
                bestCost = open.top().cost;
                // std::cout << "bestFScore: " << bestFScore << std::endl;
                if (bestCost > oldBestCost) {
                    // std::cout << "oldBestCost: " << oldBestCost << " bestCost: " <<
                    // bestCost << std::endl;
                    auto iter = open.ordered_begin();
                    auto iterEnd = open.ordered_end();
                    for (; iter != iterEnd; ++iter) {
                        int val = iter->cost;
                        if (val > oldBestCost * m_w && val <= bestCost * m_w) {
                            const HighLevelNode& n = *iter;
                            focal.push(n.handle);
                        }
                        if (val > bestCost * m_w) {
                            break;
                        }
                    }
                }
            }
#endif
// check focal list/open list consistency
#ifdef CHECK_FOCAL_LIST
            {
    // focalSet_t focalSetGolden;
    bool mismatch = false;
    const auto& top = open.top();
    int bestCost = top.cost;
    auto iter = open.ordered_begin();
    auto iterEnd = open.ordered_end();
    for (; iter != iterEnd; ++iter) {
      const auto& s = *iter;
      int val = s.cost;
      if (val <= bestCost * m_w) {
        // std::cout << "should: " << s << std::endl;
        // focalSetGolden.push(s.handle);
        if (std::find(focal.begin(), focal.end(), s.handle) ==
            focal.end()) {
          std::cout << "focal misses: " << s << std::endl;
          mismatch = true;
        }

      } else {
        if (std::find(focal.begin(), focal.end(), s.handle) !=
            focal.end()) {
          std::cout << "focalSet shouldn't have: " << s << std::endl;
          mismatch = true;
        }
        // break;
      }
    }
    assert(!mismatch);
    // assert(focalSet == focalSetGolden);
  }
#endif

            auto h = focal.top();
            HighLevelNode P = *h;
            m_env.onExpandHighLevelNode(P.cost);
            // std::cout << "expand: " << P << std::endl;

            focal.pop();
            open.erase(h);

            Conflict conflict;
            if (!m_env.getFirstConflict(P.solution, conflict)) {
                // std::cout << "done; cost: " << P.cost << std::endl;
                solution = P.solution;
                return true;
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
                newNode.LB -= newNode.solution[i].fmin;

                LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
                                          newNode.solution);
                LowLevelSearch_t lowLevel(llenv, m_w);
                bool success = lowLevel.search(initialStates[i], newNode.solution[i]);

                newNode.cost += newNode.solution[i].cost;
                newNode.LB += newNode.solution[i].fmin;
                newNode.focalHeuristic = m_env.focalHeuristic(newNode.solution);

                if (success) {
                    // std::cout << "  success. cost: " << newNode.cost << std::endl;
                    auto handle = open.push(newNode);
                    (*handle).handle = handle;
                    if (newNode.cost <= bestCost * m_w) {
                        focal.push(handle);
                    }
                }

                ++id;
            }
        }

        return false;
    }

private:
    struct HighLevelNode;

#ifdef USE_FIBONACCI_HEAP
    typedef typename boost::heap::fibonacci_heap<HighLevelNode> openSet_t;
typedef typename openSet_t::handle_type handle_t;
// typedef typename boost::heap::fibonacci_heap<fibHeapHandle_t,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#else
    typedef typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
    boost::heap::mutable_<true> >
    openSet_t;
    typedef typename openSet_t::handle_type handle_t;
// typedef typename boost::heap::d_ary_heap<fibHeapHandle_t,
// boost::heap::arity<2>, boost::heap::mutable_<true>,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#endif

    struct HighLevelNode {
        std::vector<PlanResult> solution;
        std::vector<Constraints> constraints;

        int cost;
        int LB;  // sum of fmin of solution

        int focalHeuristic;

        int id;

        handle_t handle;

        bool operator<(const HighLevelNode& n) const {
            // if (cost != n.cost)
            return cost > n.cost;
            // return id > n.id;
        }

        friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
            os << "id: " << c.id << " cost: " << c.cost << " LB: " << c.LB
               << " focal: " << c.focalHeuristic << std::endl;
            for (size_t i = 0; i < c.solution.size(); ++i) {
                os << "Agent: " << i << std::endl;
                os << " States:" << std::endl;
                for (size_t t = 0; t < c.solution[i].path.size(); ++t) {
                    os << "  " << c.solution[i].path[t].first << std::endl;
                }
                os << " Constraints:" << std::endl;
                os << c.constraints[i];
                os << " cost: " << c.solution[i].cost << std::endl;
            }
            return os;
        }
    };

    struct compareFocalHeuristic {
        bool operator()(const handle_t& h1, const handle_t& h2) const {
            // Our heap is a maximum heap, so we invert the comperator function here
            if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
                return (*h1).focalHeuristic > (*h2).focalHeuristic;
            }
            return (*h1).cost > (*h2).cost;
        }
    };

#ifdef USE_FIBONACCI_HEAP
    typedef typename boost::heap::fibonacci_heap<
  openSet_t, boost::heap::compare<compareFocalHeuristic> >
  focalSet_t;
#else
    typedef typename boost::heap::d_ary_heap<
            handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compareFocalHeuristic> >
    focalSet_t;
#endif

    struct LowLevelEnvironment {
        LowLevelEnvironment(
                Environment& env, size_t agentIdx, const Constraints& constraints,
                const std::vector<PlanResult>& solution)
                : m_env(env)
                // , m_agentIdx(agentIdx)
                // , m_constraints(constraints)
                ,
                  m_solution(solution) {
            m_env.setLowLevelContext(agentIdx, &constraints);
        }

        int admissible_heuristic(const TimeLocation& s)
        {
            return m_env.admissible_heuristic(s);
        }

        int focalStateHeuristic(const TimeLocation& s, int gScore)
        {
            return m_env.focalStateHeuristic(s, gScore, m_solution);
        }

        int focalTransitionHeuristic(const TimeLocation& s1, const TimeLocation& s2,
                                      int gScoreS1, int gScoreS2)
        {
            return m_env.focalTransitionHeuristic(s1, s2, gScoreS1, gScoreS2,
                                                  m_solution);
        }

        bool is_solution(const TimeLocation& s) { return m_env.is_solution(s); }

        void get_neighbors(const TimeLocation& s,
                           std::vector<Neighbor>& neighbors) {
            m_env.get_neighbors(s, neighbors);
        }

        void onExpandNode(const TimeLocation& s, int fScore, int gScore) {
            // std::cout << "LL expand: " << s << " fScore: " << fScore << " gScore: "
            // << gScore << std::endl;
            // m_env.onExpandLowLevelNode(s, fScore, gScore, m_agentIdx,
            // m_constraints);
            m_env.onExpandLowLevelNode(s, fScore, gScore);
        }

        void onDiscover(const TimeLocation& /*s*/, int /*fScore*/, int /*gScore*/)
        {
            // std::cout << "LL discover: " << s << std::endl;
            // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
        }

    private:
        Environment& m_env;
        // size_t m_agentIdx;
        // const Constraints& m_constraints;
        const std::vector<PlanResult>& m_solution;
    };

private:
    Environment& m_env;
    float m_w;
    typedef AStarEpsilon<LowLevelEnvironment> LowLevelSearch_t;
};




#endif // ECBS_ISOLATED_HPP
