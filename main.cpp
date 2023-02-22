#include <iostream>
#include "weighted_graph.h"

int main() {
    WeightedGraph graph;

    graph.addNode(1);
    graph.addNode(2);
    graph.addNode(3);
    graph.addNode(4);
    graph.addEdge(1, 2, 10);
    graph.addEdge(2, 3, 20);
    graph.addEdge(1, 3, 30);
    graph.addEdge(1, 4, 20);


    graph.print();
    std::cout<<"\nBFS:\n";
    graph.BFS(2);

    std::cout<<"\nDFS:\n";
    graph.DFS(1);

    std::cout<<"\nPrimMST:\n";
    graph.PrimMST();

    std::cout<<"\nDijkstra Shortest Path:\n";
    graph.DijkstraShortestPath(2);

    return 0;
}

