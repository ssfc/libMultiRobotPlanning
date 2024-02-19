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
#include <libMultiRobotPlanning/next_best_assignment.hpp>


#include "a_star.hpp"

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


template <typename Environment>
class CBSTA
{
public:
    CBSTA(Environment& environment) : m_env(environment) {}

    bool search(const std::vector<State>& initialStates,
                std::vector<PlanResult<State, Action, int> >& solution) {
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

private:
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

    struct LowLevelEnvironment
    {
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

        void get_neighbors(const State& s, std::vector<Neighbor<State, Action, int> >& neighbors)
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

    private:
        Environment& m_env;
        // size_t m_agentIdx;
        // const Constraints& m_constraints;
    };

private:
    Environment& m_env;
    typedef AStar<State, Action, LowLevelEnvironment> LowLevelSearch_t;
};

#endif  // CBS_TA_ISOLATED_HPP
