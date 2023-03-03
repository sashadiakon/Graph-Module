// weighted_graph.h
#ifndef WEIGHTED_GRAPH_H
#define WEIGHTED_GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include <queue>
#include <stack>
#include <climits>
#include <limits>
#include <fstream>

using namespace std;

class WeightedGraph {
    public:
        // Initialize graph from file
        void readFromFile(const std::string& fileName);

        // Adds a node to the graph
        void addNode(int node);

        // Adds an edge between two nodes with a weight
        void addEdge(int node1, int node2, int weight);

        // Returns the weight of the edge between two nodes
        int getWeight(int node1, int node2);

        // Returns all the nodes in the graph
        std::vector<int> getNodes();

        // Returns all the edges and their weights for a given node
        std::map<int, int> getEdges(int node);

        // Prints the graph
        void print();
        
        // BFS
        vector<int> BFS(int node);
        vector<int> DFS(int startNode);    
        void PrimMST();
        std::vector<int> DijkstraShortestPath(int startNode);
        std::vector<int> AStarSearch(int start, int goal, std::unordered_map<int, int>& heuristics);


    private:
        std::unordered_set<int> nodes;
        std::unordered_map<int, std::map<int, int>> edges;
        std::unordered_map<int, std::vector<std::pair<int, int>>> adjList;
        int numNodes;

};

#endif // WEIGHTED_GRAPH_H

