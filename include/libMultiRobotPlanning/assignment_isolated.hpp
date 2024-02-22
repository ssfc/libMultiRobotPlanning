//
// Created by take_ on 2024/2/21.
//

#ifndef ASSIGNMENT_ISOLATED_HPP
#define ASSIGNMENT_ISOLATED_HPP

#include <boost/bimap.hpp>
#include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/find_flow_cost.hpp>
#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>


using graphTraits_t = boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::bidirectionalS>;
using vertex_t = graphTraits_t::vertex_descriptor;
using edge_t = graphTraits_t::edge_descriptor;
using agentsMap_t = boost::bimap<size_t, vertex_t>;
using agentsMapEntry_t = agentsMap_t::value_type;
using tasksMap_t = boost::bimap<Location, vertex_t>;
using tasksMapEntry_t = tasksMap_t::value_type;


struct Vertex
{
    // boost::default_color_type color;
    // edge_t predecessor;
};


class Assignment
{
private:

    agentsMap_t m_agents;
    tasksMap_t m_tasks;

public:
    Assignment()
        : m_agents(),
       m_tasks(),
       m_graph(),
       m_sourceVertex(),
       m_sinkVertex()
    {
        m_sourceVertex = boost::add_vertex(m_graph);
        m_sinkVertex = boost::add_vertex(m_graph);
    }

    void clear()
    {
        // std::cout << "Asg: clear" << std::endl;
        std::set<edge_t> edgesToRemove;
        for (const auto& agent : m_agents)
        {
            auto es = boost::out_edges(agent.right, m_graph);
            for (auto eit = es.first; eit != es.second; ++eit)
            {
                if (!m_graph[*eit].isReverseEdge)
                {
                    edgesToRemove.insert(*eit);
                    edgesToRemove.insert(m_graph[*eit].reverseEdge);
                }
            }
        }

        for (const auto& e : edgesToRemove)
        {
            boost::remove_edge(e, m_graph);
        }
    }

    void set_cost(const size_t& agent, const Location& task, long cost)
    {
        // std::cout << "set_cost: " << agent << "->" << task << " cost: " << cost <<
        // std::endl;
        // Lazily create vertex for agent
        auto agentIter = m_agents.left.find(agent);
        vertex_t agentVertex;
        if (agentIter == m_agents.left.end())
        {
            agentVertex = boost::add_vertex(m_graph);
            addOrUpdateEdge(m_sourceVertex, agentVertex, 0);
            m_agents.insert(agentsMapEntry_t(agent, agentVertex));
        }
        else
        {
            agentVertex = agentIter->second;
        }

        // Lazily create vertex for task
        auto taskIter = m_tasks.left.find(task);
        vertex_t taskVertex;
        if (taskIter == m_tasks.left.end())
        {
            taskVertex = boost::add_vertex(m_graph);
            addOrUpdateEdge(taskVertex, m_sinkVertex, 0);
            m_tasks.insert(tasksMapEntry_t(task, taskVertex));
        }
        else
        {
            taskVertex = taskIter->second;
        }

        addOrUpdateEdge(agentVertex, taskVertex, cost);
    }

    // find first (optimal) solution with minimal cost
    long solve(std::map<size_t, Location>& solution)
    {
        using namespace boost;

        successive_shortest_path_nonnegative_weights(
            m_graph, m_sourceVertex, m_sinkVertex,
            boost::capacity_map(get(&Edge::capacity, m_graph))
                .residual_capacity_map(get(&Edge::residualCapacity, m_graph))
                .weight_map(get(&Edge::cost, m_graph))
                .reverse_edge_map(get(&Edge::reverseEdge, m_graph)));

        // long cost = find_flow_cost(
        //   m_graph,
        //   boost::capacity_map(get(&Edge::capacity, m_graph))
        //   .residual_capacity_map(get(&Edge::residualCapacity, m_graph))
        //   .weight_map(get(&Edge::cost, m_graph)));
        long cost = 0;

        // find solution
        solution.clear();
        auto es = out_edges(m_sourceVertex, m_graph);
        for (auto eit = es.first; eit != es.second; ++eit)
        {
            vertex_t agentVertex = target(*eit, m_graph);
            auto es2 = out_edges(agentVertex, m_graph);
            for (auto eit2 = es2.first; eit2 != es2.second; ++eit2)
            {
                if (!m_graph[*eit2].isReverseEdge)
                {
                    vertex_t taskVertex = target(*eit2, m_graph);
                    if (m_graph[*eit2].residualCapacity == 0)
                    {
                        solution[m_agents.right.at(agentVertex)] = m_tasks.right.at(taskVertex);
                        cost += m_graph[edge(agentVertex, taskVertex, m_graph).first].cost;
                        break;
                    }
                }
            }
        }

        return cost;
    }

protected:

    struct Edge
    {
        long cost;
        long capacity;
        long residualCapacity;
        edge_t reverseEdge;
        bool isReverseEdge;

        Edge()
            : cost(0),
              capacity(0),
              residualCapacity(0),
              reverseEdge(),
              isReverseEdge(false)
        {}
    };

    using graph_t = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Vertex, Edge>;

protected:
    void addOrUpdateEdge(vertex_t from, vertex_t to, long cost)
    {
        auto e = boost::edge(from, to, m_graph);
        if (e.second)
        {
            m_graph[e.first].cost = cost;
            m_graph[m_graph[e.first].reverseEdge].cost = -cost;
        }
        else
        {
            auto e1 = boost::add_edge(from, to, m_graph);
            m_graph[e1.first].cost = cost;
            m_graph[e1.first].capacity = 1;
            auto e2 = boost::add_edge(to, from, m_graph);
            m_graph[e2.first].isReverseEdge = true;
            m_graph[e2.first].cost = -cost;
            m_graph[e2.first].capacity = 0;
            m_graph[e1.first].reverseEdge = e2.first;
            m_graph[e2.first].reverseEdge = e1.first;
        }
    }

private:
    graph_t m_graph;
    vertex_t m_sourceVertex;
    vertex_t m_sinkVertex;
};


#endif  // ASSIGNMENT_ISOLATED_HPP