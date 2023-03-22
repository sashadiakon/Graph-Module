#ifndef GRAPH_H
#define GRAPH_H

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


class AStarNode {
public:
    int id;
    int fScore; // f(n) = g(n) + h(n)
    int gScore; // g(n) - cost to reach this node
    AStarNode(int id, int gScore, int fScore);
    bool operator<(const AStarNode &other) const;
};


class WeightedGraph {
    public:
        WeightedGraph();
        // Initialize graph from file
        void readFromFile(string fileName, bool directed);

        // Adds a node to the graph
        void addNode(int node);

        // Adds an edge between two nodes with a weight
        void addEdge(int node1, int node2, int weight, bool directed);

        // Returns the weight of the edge between two nodes
        int getWeight(int node1, int node2);
        unordered_map<int, vector<pair<int, int>>> getGraph();
        int  getNumNodes();
        // Returns all the nodes in the graph
        unordered_set<int> getNodes();

        // Returns all the edges and their weights for a given node
        vector<pair<int, int>> getEdges(int node);

        // Prints the graph
        void print();
        
        // BFS
        vector<int> BFS(int node);
        vector<int> DFS(int startNode);    
        vector<int> PrimMST();
        vector<vector<int>> DijkstraShortestPath(int startNode);
        vector<int> AStarSearch(int start, int goal, unordered_map<int, int>& heuristics);
        // pair<int, vector<vector<int>>> FordFulkerson(int source, int sink);


    private:
        unordered_set<int> nodes;
        unordered_map<int, vector<pair<int, int>>> adjList;
        int dfsFordFulkerson(int u, int t, int flow, vector<int>& parent, vector<vector<int>>& residualGraph);
        int numNodes;        


};




#endif // WEIGHTED_GRAPH_H

