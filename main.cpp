#include <iostream>
#include "weighted_graph.h"

int main() {
    WeightedGraph graph;
    graph.readFromFile("input.txt", false);


    graph.print();

    cout<<"\nBFS:\n";
    vector<int> a = graph.BFS(2);

    cout<<"\nDFS:\n";
    vector<int> b = graph.DFS(1);

    cout<<"\nPrimMST:\n";
    vector<int> c = graph.PrimMST();

    cout<<"\nDijkstra Shortest Path:\n";
    vector<int> d = graph.DijkstraShortestPath(2);
    for (auto node: d){
        cout<<node<<" ";
    }

    cout<<"\nA* Alg:\n";
    unordered_map<int, int> euclideanDist = {
    {0, 9},
    {1, 7},
    {2, 6},
    {3, 5},
    {4, 7},
    {5, 3},
    {6, 2},
    {7, 0}
    } ;
    vector<int> f = graph.AStarSearch(1, 7, euclideanDist);
    for (auto node: f){
        cout<<node<<" ";
    }
    cout<<"\nFordFulkerson alg:\n";
    auto k = graph.FordFulkerson(0, 7);
    cout<<k.first<<"\n";
    // for (auto s : k.second){
    //     for (int a: s){
    //         cout<<a<<" ";
    //     }
    //     cout<<"\n";
    // }
    return 0;
}

 