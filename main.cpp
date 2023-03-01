#include <iostream>
#include "weighted_graph.h"

int main() {
    WeightedGraph graph;

    graph.addNode(1);
    graph.addNode(2);
    graph.addNode(3);
    graph.addNode(4);
    graph.addNode(5);
    
    graph.addEdge(1, 2, 10);
    graph.addEdge(1, 3, 15);
    graph.addEdge(1, 4, 8);
    graph.addEdge(2, 4, 25);
    graph.addEdge(2, 3, 10);
    graph.addEdge(3, 5, 13);
    graph.addEdge(3, 4, 9);
    graph.addEdge(4, 5, 11);
        


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

