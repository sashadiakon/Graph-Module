#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "graph.h"

using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(mygraph, m) {
    m.doc() = "optional module docstring";
    py::class_<WeightedGraph>(m, "WeightedGraph")
        .def(py::init<>())
        .def("add_node", &WeightedGraph::addNode)
        .def("add_edge", &WeightedGraph::addEdge)
        .def("read_from_file", &WeightedGraph::readFromFile)
        .def("get_graph", &WeightedGraph::getGraph)
        .def("get_weight", &WeightedGraph::getWeight)
        .def("get_nodes", &WeightedGraph::getNodes)
        .def("get_edges", &WeightedGraph::getEdges)
        .def("print", &WeightedGraph::print)    

        .def("bfs", &WeightedGraph::BFS)
        .def("dfs", &WeightedGraph::DFS)
        .def("prims", &WeightedGraph::PrimMST)
        .def("dijkstra_shortest_path", &WeightedGraph::DijkstraShortestPath)
        .def("a_star_search", &WeightedGraph::AStarSearch)
        .def("__len__", &WeightedGraph::getNumNodes);
}
