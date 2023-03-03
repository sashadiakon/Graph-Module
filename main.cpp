#include <iostream>
#include "weighted_graph.h"

int main() {
    WeightedGraph graph;

    graph.readFromFile("input.txt");


    graph.print();
    std::cout<<"\nBFS:\n";
    graph.BFS(2);

    std::cout<<"\nDFS:\n";
    graph.DFS(1);

    std::cout<<"\nPrimMST:\n";
    graph.PrimMST();

    std::cout<<"\nDijkstra Shortest Path:\n";
    graph.DijkstraShortestPath(2);

    // graph.AStarSearch(1, 4, );


    return 0;
}

