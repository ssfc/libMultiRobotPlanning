//
// Created by take_ on 2024/2/19.
//

#ifndef CBS_TA_ISOLATED_HPP
#define CBS_TA_ISOLATED_HPP

#include <map>

#include "a_star.hpp"

namespace libMultiRobotPlanning {

/*!
  \example cbs_ta.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief Conflict-Based-Search with Optimal Task Assignment (CBS-TA) algorithm
to find tasks and collision-free paths jointly, minimizing sum-of-cost.

This class implements the Conflict-Based-Search with Optimal Task Assignment
(CBS-TA) algorithm.
This algorithm assigns tasks and finds collision-free path for multiple agents
with start and
goal locations given for each agent.
CBS-TA is an extension of the CBS algorithms, operating in a search forest
rather
than a search tree (where each root node refers to a possible assignment).
CBS-TA is optimal with respect to the sum-of-individual costs.

Details of the algorithm can be found in the following paper:\n
W. Hönig, S. Kiesel, A. Tinka, J. W. Durham, and N. Ayanian.\n
"Conflict-Based Search with Optimal Task Assignment",\n
In Proc. of the 17th International Conference on Autonomous Agents and
Multiagent Systems (AAMAS)\n
Stockholm, Sweden, July 2018.

The underlying A* can either use a fibonacci heap, or a d-ary heap.
The latter is the default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap
instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\param Task Custom task type to be used for assignment.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:
  - `void setLowLevelContext(size_t agentIdx, const Constraints* constraints)`\n
    Set the current context to a particular agent with the given set of
constraints

  - `Cost admissible_heuristic(const State& s)`\n
    Admissible heuristic. Needs to take current context into account.

  - `bool is_solution(const State& s)`\n
    Return true if the given state is a goal state for the current agent.

  - `void get_neighbors(const State& s, std::vector<Neighbor<State, Action, int>
>& neighbors)`\n
    Fill the list of neighboring state for the given state s and the current
agent.

  - `bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&
solution, Conflict& result)`\n
    Finds the first conflict for the given solution for each agent. Return true
if a conflict was found and false otherwise.

  - `void createConstraintsFromConflict(const Conflict& conflict,
std::map<size_t, Constraints>& constraints)`\n
    Create a list of constraints for the given conflict.

  - `void onExpandHighLevelNode(Cost cost)`\n
    This function is called on every high-level expansion and can be used for
statistical purposes.

  - `void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every low-level expansion and can be used for
statistical purposes.
*/
template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Task, typename Environment>
class CBSTA {
   public:
    CBSTA(Environment& environment) : m_env(environment) {}

    bool search(const std::vector<State>& initialStates,
                std::vector<PlanResult<State, Action, Cost> >& solution) {
        HighLevelNode start;
        size_t numAgents = initialStates.size();
        start.solution.resize(numAgents);
        start.constraints.resize(numAgents);
        start.cost = 0;
        start.id = 0;
        start.isRoot = true;
        m_env.nextTaskAssignment(start.tasks);

        for (size_t i = 0; i < initialStates.size(); ++i) {
            // if (   i < solution.size()
            //     && solution[i].path.size() > 1) {
            //   start.solution[i] = solution[i];
            //   std::cout << "use existing solution for agent: " << i << std::endl;
            // } else {
            bool success = false;
            if (!start.tasks.empty()) {
                LowLevelEnvironment llenv(m_env, i, start.constraints[i],
                                          start.task(i));
                LowLevelSearch_t lowLevel(llenv);
                success = lowLevel.a_star_search(initialStates[i], start.solution[i]);
            }
            if (!success) {
                return false;
            }
            // }
            start.cost += start.solution[i].cost;
        }

        // std::priority_queue<HighLevelNode> open;
        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                         boost::heap::mutable_<true> >
            open;

        auto handle = open.push(start);
        (*handle).handle = handle;

        solution.clear();
        int id = 1;
        while (!open.empty()) {
            HighLevelNode P = open.top();
            m_env.onExpandHighLevelNode(P.cost);
            // std::cout << "expand: " << P << std::endl;

            open.pop();

            Conflict conflict;
            if (!m_env.getFirstConflict(P.solution, conflict)) {
                std::cout << "done; cost: " << P.cost << std::endl;
                solution = P.solution;
                return true;
            }

            if (P.isRoot) {
                // std::cout << "root node expanded; add new root" << std::endl;
                HighLevelNode n;
                m_env.nextTaskAssignment(n.tasks);

                if (n.tasks.size() > 0) {
                    n.solution.resize(numAgents);
                    n.constraints.resize(numAgents);
                    n.cost = 0;
                    n.id = id;
                    n.isRoot = true;

                    bool allSuccessful = true;
                    for (size_t i = 0; i < numAgents; ++i) {
                        LowLevelEnvironment llenv(m_env, i, n.constraints[i], n.task(i));
                        LowLevelSearch_t lowLevel(llenv);
                        bool success = lowLevel.a_star_search(initialStates[i], n.solution[i]);
                        if (!success) {
                            allSuccessful = false;
                            break;
                        }
                        n.cost += n.solution[i].cost;
                    }
                    if (allSuccessful) {
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
            for (const auto& c : constraints) {
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

                if (success) {
                    // std::cout << "  success. cost: " << newNode.cost << std::endl;
                    auto handle = open.push(newNode);
                    (*handle).handle = handle;
                }

                ++id;
            }
        }

        return false;
    }

   private:
    struct HighLevelNode {
        std::vector<PlanResult<State, Action, Cost> > solution;
        std::vector<Constraints> constraints;
        std::map<size_t, Task> tasks; // maps from index to task (and does not contain an entry if no task was assigned)

        Cost cost;

        int id;
        bool isRoot;

        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                         boost::heap::mutable_<true> >::handle_type
            handle;

        bool operator<(const HighLevelNode& n) const {
            // if (cost != n.cost)
            return cost > n.cost;
            // return id > n.id;
        }

        Task* task(size_t idx)
        {
            Task* task = nullptr;
            auto iter = tasks.find(idx);
            if (iter != tasks.end()) {
                task = &iter->second;
            }
            return task;
        }

        friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
            os << "id: " << c.id << " cost: " << c.cost << std::endl;
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

    struct LowLevelEnvironment {
        LowLevelEnvironment(Environment& env, size_t agentIdx,
                            const Constraints& constraints, const Task* task)
            : m_env(env)
        // , m_agentIdx(agentIdx)
        // , m_constraints(constraints)
        {
            m_env.setLowLevelContext(agentIdx, &constraints, task);
        }

        Cost admissible_heuristic(const State& s) {
            return m_env.admissible_heuristic(s);
        }

        bool is_solution(const State& s) { return m_env.is_solution(s); }

        void get_neighbors(const State& s,
                           std::vector<Neighbor<State, Action, Cost> >& neighbors) {
            m_env.get_neighbors(s, neighbors);
        }

        void onExpandNode(const State& s, Cost fScore, Cost gScore) {
            // std::cout << "LL expand: " << s << std::endl;
            m_env.onExpandLowLevelNode(s, fScore, gScore);
        }

        void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
            // std::cout << "LL discover: " << s << std::endl;
            // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
        }

       private:
        Environment& m_env;
        // size_t m_agentIdx;
        // const Constraints& m_constraints;
    };

   private:
    Environment& m_env;
    typedef AStar<State, Action, LowLevelEnvironment> LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning

#endif  // CBS_TA_ISOLATED_HPP
