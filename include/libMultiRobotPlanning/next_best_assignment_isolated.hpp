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
    std::set<std::pair<size_t, Location> > solution_include;  // enforced assignment
    std::set<std::pair<size_t, Location> > O;  // invalid assignments
    std::set<size_t> Iagents;  // agents that must have an assignment
    std::set<size_t> Oagents;  // agents that should not have an assignment
    std::map<size_t, Location> solution;
    long cost;

public:
    ASGNode()
        : solution_include(),
          O(),
          Iagents(),
          Oagents(),
          solution(),
          cost(0)
    {}

    bool operator<(const ASGNode& asg_node) const
    {
        // Our heap is a maximum heap, so we invert the comperator function here
        return cost > asg_node.cost;
    }

    friend std::ostream& operator<<(std::ostream& os, const ASGNode& asg_node)
    {
        os << "ASGNode with cost: " << asg_node.cost << std::endl;
        os << "  solution_include: ";

        for (const auto& c : asg_node.solution_include)
        {
            os << c.first << "->" << c.second << ",";
        }

        os << std::endl;
        os << "  O: ";

        for (const auto& c : asg_node.O)
        {
            os << c.first << "->" << c.second << ",";
        }

        os << std::endl;
        os << "  Iagents: ";

        for (const auto& c : asg_node.Iagents)
        {
            os << c << ",";
        }

        os << std::endl;
        os << "  Oagents: ";

        for (const auto& c : asg_node.Oagents)
        {
            os << c << ",";
        }

        os << std::endl;
        os << "  solution: ";

        for (const auto& c : asg_node.solution)
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
    std::vector<size_t> agents_vec;
    std::set<size_t> agents_set;
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
        if (agents_set.find(agent) == agents_set.end())
        {
            agents_set.insert(agent);
            agents_vec.emplace_back(agent);
        }
        // m_tasksSet.insert(task);
    }

    size_t get_num_matching(const std::map<size_t, Location>& solution)
    {
        return solution.size();
    }

    // solution_include enforces that the respective pair is part of the solution
    // O enforces that the respective pair is not part of the solution
    // Iagents enforces that these agents must have a task assignment
    // Oagents enforces that these agents should not have any task assignment
    long constrained_matching(const std::set<std::pair<size_t, Location> >& solution_include,
                             const std::set<std::pair<size_t, Location> >& O,
                             const std::set<size_t>& Iagents,
                             const std::set<size_t>& Oagents,
                             std::map<size_t, Location>& solution)
    {
        // prepare assignment problem

        assignment.clear();

        for (const auto& c : solution_include)
        {
            if (Oagents.find(c.first) == Oagents.end())
            {
                assignment.set_cost(c.first, c.second, 0);
            }
        }

        for (const auto& c : map_cost)
        {
            if (O.find(c.first) == O.end() && solution_include.find(c.first) == solution_include.end() &&
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

        // std::cout << "constrained_matching: internal Solution: " << std::endl;
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
        // check that solution_include constraints have been fulfilled
        for (const auto& c : solution_include)
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
        const std::set<std::pair<size_t, Location> > solution_include, O;
        const std::set<size_t> Iagents, Oagents;
        ASGNode asg_node;
        asg_node.cost = constrained_matching(solution_include, O, Iagents, Oagents, asg_node.solution);
        asg_open.emplace(asg_node);
        num_matching = get_num_matching(asg_node.solution);
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

        std::set<size_t> fixed_agents;
        for (const auto& c : next.solution_include)
        {
            fixed_agents.insert(c.first);
        }

        // prepare for next query
        for (size_t i = 0; i < agents_vec.size(); ++i)
        {
            if (fixed_agents.find(agents_vec[i]) == fixed_agents.end())
            {
                ASGNode asg_node;
                asg_node.solution_include = next.solution_include;
                asg_node.O = next.O;
                asg_node.Iagents = next.Iagents;
                asg_node.Oagents = next.Oagents;
                // fix assignment for agents 0...i
                for (size_t j = 0; j < i; ++j)
                {
                    const size_t& agent = agents_vec[j];
                    // asg_node.solution_include.insert(std::make_pair<>(agent, next.solution.at(agent)));
                    const auto iter = solution.find(agent);
                    if (iter != solution.end())
                    {
                        asg_node.solution_include.insert(std::make_pair<>(agent, iter->second));
                    }
                    else
                    {
                        // this agent should keep having no solution =>
                        // enforce that no task is allowed
                        asg_node.Oagents.insert(agent);
                        // for (const auto& task : m_tasksSet) {
                        //   asg_node.O.insert(std::make_pair<>(agent, task));
                        // }
                    }
                }
                // asg_node.O.insert(
                //     std::make_pair<>(agents_vec[i], next.solution.at(agents_vec[i])));
                const auto iter = solution.find(agents_vec[i]);
                if (iter != solution.end())
                {
                    asg_node.O.insert(std::make_pair<>(agents_vec[i], iter->second));
                }
                else
                {
                    // this agent should have a solution next
                    // std::cout << "should have sol: " << agents_vec[i] << std::endl;
                    asg_node.Iagents.insert(agents_vec[i]);
                }
                // std::cout << " consider adding: " << asg_node << std::endl;
                asg_node.cost = constrained_matching(asg_node.solution_include, asg_node.O,
                                                     asg_node.Iagents, asg_node.Oagents,
                                                     asg_node.solution);

                if (asg_node.solution.size() > 0)
                {
                    asg_open.push(asg_node);
                    // std::cout << "add: " << asg_node << std::endl;
                }
            }
        }

        return result;
    }
};


#endif  // NEXT_BEST_ASSIGNMENT_ISOLATED_HPP
