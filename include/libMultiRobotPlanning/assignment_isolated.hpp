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


struct Edge
{
    long cost;
    long capacity;
    long residual_capacity;
    edge_t reverse_edge;
    bool is_reverse_edge;

    Edge()
        : cost(0),
          capacity(0),
          residual_capacity(0),
          reverse_edge(),
          is_reverse_edge(false)
    {}
};


using graph_t = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, Vertex, Edge>;


class Assignment
{
private:
    agentsMap_t agents;
    tasksMap_t tasks;
    graph_t graph;
    vertex_t source_vertex;
    vertex_t m_sinkVertex;

public:
    Assignment()
        : agents(),
       tasks(),
       graph(),
       source_vertex(),
       m_sinkVertex()
    {
        source_vertex = boost::add_vertex(graph);
        m_sinkVertex = boost::add_vertex(graph);
    }

    void clear()
    {
        // std::cout << "Asg: clear" << std::endl;
        std::set<edge_t> edgesToRemove;
        for (const auto& agent : agents)
        {
            auto es = boost::out_edges(agent.right, graph);
            for (auto eit = es.first; eit != es.second; ++eit)
            {
                if (!graph[*eit].is_reverse_edge)
                {
                    edgesToRemove.insert(*eit);
                    edgesToRemove.insert(graph[*eit].reverse_edge);
                }
            }
        }

        for (const auto& e : edgesToRemove)
        {
            boost::remove_edge(e, graph);
        }
    }

    void set_cost(const size_t& agent, const Location& task, long cost)
    {
        // std::cout << "set_cost: " << agent << "->" << task << " cost: " << cost <<
        // std::endl;
        // Lazily create vertex for agent
        auto agentIter = agents.left.find(agent);
        vertex_t agentVertex;
        if (agentIter == agents.left.end())
        {
            agentVertex = boost::add_vertex(graph);
            addOrUpdateEdge(source_vertex, agentVertex, 0);
            agents.insert(agentsMapEntry_t(agent, agentVertex));
        }
        else
        {
            agentVertex = agentIter->second;
        }

        // Lazily create vertex for task
        auto taskIter = tasks.left.find(task);
        vertex_t taskVertex;
        if (taskIter == tasks.left.end())
        {
            taskVertex = boost::add_vertex(graph);
            addOrUpdateEdge(taskVertex, m_sinkVertex, 0);
            tasks.insert(tasksMapEntry_t(task, taskVertex));
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
            graph, source_vertex, m_sinkVertex,
            boost::capacity_map(get(&Edge::capacity, graph))
                .residual_capacity_map(get(&Edge::residual_capacity, graph))
                .weight_map(get(&Edge::cost, graph))
                .reverse_edge_map(get(&Edge::reverse_edge, graph)));

        // long cost = find_flow_cost(
        //   graph,
        //   boost::capacity_map(get(&Edge::capacity, graph))
        //   .residual_capacity_map(get(&Edge::residual_capacity, graph))
        //   .weight_map(get(&Edge::cost, graph)));
        long cost = 0;

        // find solution
        solution.clear();
        auto es = out_edges(source_vertex, graph);
        for (auto eit = es.first; eit != es.second; ++eit)
        {
            vertex_t agentVertex = target(*eit, graph);
            auto es2 = out_edges(agentVertex, graph);
            for (auto eit2 = es2.first; eit2 != es2.second; ++eit2)
            {
                if (!graph[*eit2].is_reverse_edge)
                {
                    vertex_t taskVertex = target(*eit2, graph);
                    if (graph[*eit2].residual_capacity == 0)
                    {
                        solution[agents.right.at(agentVertex)] = tasks.right.at(taskVertex);
                        cost += graph[edge(agentVertex, taskVertex, graph).first].cost;
                        break;
                    }
                }
            }
        }

        return cost;
    }


protected:
    void addOrUpdateEdge(vertex_t from, vertex_t to, long cost)
    {
        auto e = boost::edge(from, to, graph);
        if (e.second)
        {
            graph[e.first].cost = cost;
            graph[graph[e.first].reverse_edge].cost = -cost;
        }
        else
        {
            auto e1 = boost::add_edge(from, to, graph);
            graph[e1.first].cost = cost;
            graph[e1.first].capacity = 1;
            auto e2 = boost::add_edge(to, from, graph);
            graph[e2.first].is_reverse_edge = true;
            graph[e2.first].cost = -cost;
            graph[e2.first].capacity = 0;
            graph[e1.first].reverse_edge = e2.first;
            graph[e2.first].reverse_edge = e1.first;
        }
    }
};


#endif  // ASSIGNMENT_ISOLATED_HPP
