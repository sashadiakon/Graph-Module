#include <iostream>
#include "weighted_graph.h"

int main() {
    WeightedGraph graph;

    graph.readFromFile("input.txt");


    graph.print();
    std::cout<<"\nBFS:\n";
    vector<int> a = graph.BFS(2);
    std::cout<<"\nDFS:\n";
    vector<int> b = graph.DFS(1);
    std::cout<<"\nPrimMST:\n";
    vector<int> c = graph.PrimMST();

    std::cout<<"\nDijkstra Shortest Path:\n";
    graph.DijkstraShortestPath(2);

    // graph.AStarSearch(1, 4, );


    return 0;
}

