//
// Created by take_ on 2024/2/21.
//

#ifndef NEXT_BEST_ASSIGNMENT_ISOLATED_HPP
#define NEXT_BEST_ASSIGNMENT_ISOLATED_HPP

#include <queue>
#include <set>

#include "assignment_isolated.hpp"


class ASGNode
{
public:
    std::set<std::pair<size_t, Location> > I;  // enforced assignment
    std::set<std::pair<size_t, Location> > O;  // invalid assignments
    std::set<size_t> Iagents;  // agents that must have an assignment
    std::set<size_t> Oagents;  // agents that should not have an assignment
    std::map<size_t, Location> solution;
    long cost;

public:
    ASGNode()
        : I(),
          O(),
          Iagents(),
          Oagents(),
          solution(),
          cost(0)
    {}

    bool operator<(const ASGNode& node) const
    {
        // Our heap is a maximum heap, so we invert the comperator function here
        return cost > node.cost;
    }

    friend std::ostream& operator<<(std::ostream& os, const ASGNode& node)
    {
        os << "ASGNode with cost: " << node.cost << std::endl;
        os << "  I: ";

        for (const auto& c : node.I)
        {
            os << c.first << "->" << c.second << ",";
        }

        os << std::endl;
        os << "  O: ";

        for (const auto& c : node.O)
        {
            os << c.first << "->" << c.second << ",";
        }

        os << std::endl;
        os << "  Iagents: ";

        for (const auto& c : node.Iagents)
        {
            os << c << ",";
        }

        os << std::endl;
        os << "  Oagents: ";

        for (const auto& c : node.Oagents)
        {
            os << c << ",";
        }

        os << std::endl;
        os << "  solution: ";

        for (const auto& c : node.solution)
        {
            os << "    " << c.first << "->" << c.second << std::endl;
        }

        os << std::endl;

        return os;
    }
};


class NextBestAssignment
{
private:
    Assignment assignment;
    std::map<std::pair<size_t, Location>, long> map_cost;
    std::vector<size_t> m_agentsVec;
    std::set<size_t> m_agentsSet;
    // std::set<Location> m_tasksSet;
    // size_t m_numAgents;
    // size_t m_numTasks;
    // std::vector<long> m_costMatrix;
    std::priority_queue<ASGNode> asg_open;
    size_t num_matching;

public:
    NextBestAssignment(const Assignment& assignment = Assignment())
        : assignment(assignment),
          map_cost(),
          asg_open(),
          num_matching(0)
    {}

    void set_cost(const size_t& agent, const Location& task, long cost)
    {
        // std::cout << "set_cost: " << agent << "->" << task << ": " << cost <<
        // std::endl;
        map_cost[std::make_pair<>(agent, task)] = cost;
        if (m_agentsSet.find(agent) == m_agentsSet.end())
        {
            m_agentsSet.insert(agent);
            m_agentsVec.emplace_back(agent);
        }
        // m_tasksSet.insert(task);
    }

    size_t get_num_matching(const std::map<size_t, Location>& solution)
    {
        return solution.size();
    }

    // I enforces that the respective pair is part of the solution
    // O enforces that the respective pair is not part of the solution
    // Iagents enforces that these agents must have a task assignment
    // Oagents enforces that these agents should not have any task assignment
    long constrainedMatching(const std::set<std::pair<size_t, Location> >& I,
                             const std::set<std::pair<size_t, Location> >& O,
                             const std::set<size_t>& Iagents,
                             const std::set<size_t>& Oagents,
                             std::map<size_t, Location>& solution)
    {
        // prepare assignment problem

        assignment.clear();

        for (const auto& c : I)
        {
            if (Oagents.find(c.first) == Oagents.end())
            {
                assignment.set_cost(c.first, c.second, 0);
            }
        }

        for (const auto& c : map_cost)
        {
            if (O.find(c.first) == O.end() && I.find(c.first) == I.end() &&
                Oagents.find(c.first.first) == Oagents.end())
            {
                long costOffset = 1e9;// TODO: what is a good value here?
                // all agents that should have any solution will get a lower cost offset
                // enforcing this agents inclusion in the result
                if (Iagents.find(c.first.first) != Iagents.end())
                {
                    costOffset = 0;
                }
                assignment.set_cost(c.first.first, c.first.second, c.second + costOffset);
            }
        }

        assignment.solve(solution);
        size_t matching = get_num_matching(solution);

        // std::cout << "constrainedMatching: internal Solution: " << std::endl;
        // for (const auto& c : solution) {
        //   std::cout << "    " << c.first << "->" << c.second << std::endl;
        // }

        // check if all agents in Iagents have an assignment as requested
        bool solution_valid = true;
        for (const auto& agent : Iagents)
        {
            if (solution.find(agent) == solution.end())
            {
                solution_valid = false;
                break;
            }
        }
        // check that I constraints have been fulfilled
        for (const auto& c : I)
        {
            const auto& iter = solution.find(c.first);
            if (iter == solution.end() || !(iter->second == c.second))
            {
                solution_valid = false;
                break;
            }
        }

        if (!solution_valid || matching < num_matching)
        {
            solution.clear();
            return std::numeric_limits<long>::max();
        }

        return get_cost(solution);
    }

    // find first (optimal) solution with minimal cost
    void solve()
    {
        const std::set<std::pair<size_t, Location> > I, O;
        const std::set<size_t> Iagents, Oagents;
        ASGNode node;
        node.cost = constrainedMatching(I, O, Iagents, Oagents, node.solution);
        asg_open.emplace(node);
        num_matching = get_num_matching(node.solution);
    }

    long get_cost(const std::map<size_t, Location>& solution)
    {
        long result = 0;
        for (const auto& entry : solution)
        {
            result += map_cost.at(entry);
        }

        return result;
    }

    // find next solution
    long find_next_solution(std::map<size_t, Location>& solution)
    {
        solution.clear();
        if (asg_open.empty())
        {
            return std::numeric_limits<long>::max();
        }

        const ASGNode next = asg_open.top();
        // std::cout << "next: " << next << std::endl;
        asg_open.pop();

        for (const auto& entry : next.solution)
        {
            solution.insert(std::make_pair(entry.first, entry.second));
        }
        long result = next.cost;

        std::set<size_t> fixedAgents;
        for (const auto& c : next.I)
        {
            fixedAgents.insert(c.first);
        }

        // prepare for next query
        for (size_t i = 0; i < m_agentsVec.size(); ++i)
        {
            if (fixedAgents.find(m_agentsVec[i]) == fixedAgents.end())
            {
                ASGNode node;
                node.I = next.I;
                node.O = next.O;
                node.Iagents = next.Iagents;
                node.Oagents = next.Oagents;
                // fix assignment for agents 0...i
                for (size_t j = 0; j < i; ++j)
                {
                    const size_t& agent = m_agentsVec[j];
                    // node.I.insert(std::make_pair<>(agent, next.solution.at(agent)));
                    const auto iter = solution.find(agent);
                    if (iter != solution.end())
                    {
                        node.I.insert(std::make_pair<>(agent, iter->second));
                    }
                    else
                    {
                        // this agent should keep having no solution =>
                        // enforce that no task is allowed
                        node.Oagents.insert(agent);
                        // for (const auto& task : m_tasksSet) {
                        //   node.O.insert(std::make_pair<>(agent, task));
                        // }
                    }
                }
                // node.O.insert(
                //     std::make_pair<>(m_agentsVec[i], next.solution.at(m_agentsVec[i])));
                const auto iter = solution.find(m_agentsVec[i]);
                if (iter != solution.end())
                {
                    node.O.insert(std::make_pair<>(m_agentsVec[i], iter->second));
                }
                else
                {
                    // this agent should have a solution next
                    // std::cout << "should have sol: " << m_agentsVec[i] << std::endl;
                    node.Iagents.insert(m_agentsVec[i]);
                }
                // std::cout << " consider adding: " << node << std::endl;
                node.cost = constrainedMatching(node.I, node.O, node.Iagents, node.Oagents, node.solution);
                if (node.solution.size() > 0)
                {
                    asg_open.push(node);
                    // std::cout << "add: " << node << std::endl;
                }
            }
        }

        return result;
    }
};


#endif  // NEXT_BEST_ASSIGNMENT_ISOLATED_HPP
