//
// Created by take_ on 2024/2/19.
//

#ifndef READ_FILENAME_PY_SHORTEST_PATH_HEURISTIC_HPP
#define READ_FILENAME_PY_SHORTEST_PATH_HEURISTIC_HPP

#include <fstream>
#include <iostream>
#include <unordered_set>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/graphviz.hpp>

class ShortestPathHeuristic {
   public:
    ShortestPathHeuristic(size_t dimx, size_t dimy,
                          const std::unordered_set<Location>& obstacles)
        : m_shortestDistance(nullptr), num_columns(dimx), num_rows(dimy) {
        searchGraph_t searchGraph;

        // add vertices
        for (size_t x = 0; x < dimx; ++x) {
            for (size_t y = 0; y < dimy; ++y) {
                boost::add_vertex(searchGraph);
            }
        }

        // add edges
        for (size_t x = 0; x < dimx; ++x) {
            for (size_t y = 0; y < dimy; ++y) {
                Location l(x, y);
                if (obstacles.find(l) == obstacles.end()) {
                    Location right(x + 1, y);
                    if (x < dimx - 1 && obstacles.find(right) == obstacles.end()) {
                        auto e =
                            boost::add_edge(locToVert(l), locToVert(right), searchGraph);
                        searchGraph[e.first].weight = 1;
                    }
                    Location below(x, y + 1);
                    if (y < dimy - 1 && obstacles.find(below) == obstacles.end()) {
                        auto e =
                            boost::add_edge(locToVert(l), locToVert(below), searchGraph);
                        searchGraph[e.first].weight = 1;
                    }
                }
            }
        }

        writeDotFile(searchGraph, "searchGraph.dot");

        m_shortestDistance = new distanceMatrix_t(boost::num_vertices(searchGraph));
        distanceMatrixMap_t distanceMap(*m_shortestDistance, searchGraph);
        // The following generates a clang-tidy error, see
        // https://svn.boost.org/trac10/ticket/10830
        boost::floyd_warshall_all_pairs_shortest_paths(
            searchGraph, distanceMap,
            boost::weight_map(boost::get(&Edge::weight, searchGraph)));
    }

    ~ShortestPathHeuristic() { delete m_shortestDistance; }

    int getValue(const Location& a, const Location& b) {
        vertex_t idx1 = locToVert(a);
        vertex_t idx2 = locToVert(b);
        return (*m_shortestDistance)[idx1][idx2];
    }

   private:
    size_t locToVert(const Location& l) const { return l.x + num_columns * l.y; }

    Location idxToLoc(size_t idx) {
        int x = idx % num_columns;
        int y = idx / num_columns;
        return Location(x, y);
    }

   private:
    typedef boost::adjacency_list_traits<boost::vecS, boost::vecS,
                                         boost::undirectedS>
        searchGraphTraits_t;
    typedef searchGraphTraits_t::vertex_descriptor vertex_t;
    typedef searchGraphTraits_t::edge_descriptor edge_t;

    struct Vertex {};

    struct Edge {
        int weight;
    };

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                  Vertex, Edge>
        searchGraph_t;
    typedef boost::exterior_vertex_property<searchGraph_t, int>
        distanceProperty_t;
    typedef distanceProperty_t::matrix_type distanceMatrix_t;
    typedef distanceProperty_t::matrix_map_type distanceMatrixMap_t;

    class VertexDotWriter {
       public:
        explicit VertexDotWriter(const searchGraph_t& graph, size_t dimx)
            : m_graph(graph), num_columns(dimx) {}

        void operator()(std::ostream& out, const vertex_t& v) const {
            static const float DX = 100;
            static const float DY = 100;
            out << "[label=\"";
            int x = v % num_columns;
            int y = v / num_columns;
            out << "\" pos=\"" << x * DX << "," << y * DY << "!\"]";
        }

       private:
        const searchGraph_t& m_graph;
        size_t num_columns;
    };

    class EdgeDotWriter {
       public:
        explicit EdgeDotWriter(const searchGraph_t& graph) : m_graph(graph) {}

        void operator()(std::ostream& out, const edge_t& e) const {
            out << "[label=\"" << m_graph[e].weight << "\"]";
        }

       private:
        const searchGraph_t& m_graph;
    };

   private:
    void writeDotFile(const searchGraph_t& graph, const std::string& fileName) {
        VertexDotWriter vw(graph, num_columns);
        EdgeDotWriter ew(graph);
        std::ofstream dotFile(fileName);
        boost::write_graphviz(dotFile, graph, vw, ew);
    }

   private:
    distanceMatrix_t* m_shortestDistance;
    size_t num_columns;
    size_t num_rows;
};


#endif  // READ_FILENAME_PY_SHORTEST_PATH_HEURISTIC_HPP
