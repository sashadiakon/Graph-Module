#include "graph.h"
using namespace std;

WeightedGraph::WeightedGraph() {
    numNodes = 0;
}

void WeightedGraph::addNode(int node) {
    adjList[node] = vector<pair<int, int>>();
    nodes.insert(node);
    numNodes ++;
}

int WeightedGraph::getNumNodes()  {
    return numNodes;
}

void WeightedGraph::addEdge(int node1, int node2, int weight, bool directed = false) {
    adjList[node1].push_back({node2, weight});
    if (!directed){
        adjList[node2].push_back({node1, weight});
    }
    }


void WeightedGraph::readFromFile(string fileName, bool directed = false) {
    ifstream inFile(fileName);
    if (!inFile) {
        cerr << "Error: Failed to open file " << fileName << endl;
        return;
    }

    int numEdges;
    inFile >> numNodes >> numEdges;

    for (int i = 0; i < numNodes; ++i) {
        addNode(i);
        numNodes --; // as we add 1 in addNote method
    }

    for (int i = 0; i < numEdges; ++i) {
        int source, dest, weight;
        inFile >> source >> dest >> weight;
        addEdge(source, dest, weight, directed);
    }

    inFile.close();
}



void WeightedGraph::print() {
    for (const auto &node : adjList) {
        cout << node.first << ": ";
        for (const auto &neighbor : node.second) {
            cout << neighbor.first << "(" << neighbor.second << ") ";
        }
        cout << endl;
    }
}

vector<int> WeightedGraph::BFS(int start) {
    unordered_map<int, bool> visited;
    vector<int> path;
    for (const auto &node : adjList) {
        visited[node.first] = false;
    }

    queue<int> q;
    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        int node = q.front();
        q.pop();

        cout << node << " ";
        path.push_back(node);

        for (const auto &neighbor : adjList[node]) {
          if (!visited[neighbor.first]) {
            q.push(neighbor.first);
            visited[neighbor.first] = true;
          }
        }
    }
    return path;
}

vector<int> WeightedGraph::DFS(int startNode) {
    stack<int> stack;
    unordered_map<int, bool> visited;
    vector<int> path;
    for (auto &node : adjList) {

        visited[node.first] = false;
    }
    stack.push(startNode);
    while (!stack.empty()) {
        int node = stack.top();
        stack.pop();
        if (!visited[node]) {
        visited[node] = true;
        cout << node << " ";
        path.push_back(node);
        for (auto &neighbor : adjList[node]) {
            stack.push(neighbor.first);
        }
        }
    }
    cout << endl;
    return path;
}

vector<int> WeightedGraph::PrimMST() {
    vector<int> key(numNodes, INT_MAX);
    vector<int> parent(numNodes, -1);
    vector<bool> inMST(numNodes, false);
    key[0] = 0;
    priority_queue<pair<int, int>, vector<pair<int, int>>,
                        greater<pair<int, int>>>pq;
    pq.push({0, 0});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        inMST[u] = true;
        for (auto &neighbor : adjList[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;
            if (!inMST[v] && key[v] > weight) {
                key[v] = weight;
                pq.push({key[v], v});
                parent[v] = u;
            }
        }
    }

    for (int i = 1; i < numNodes; i++) {
        cout << parent[i] << " -> " << i << endl;
    }
    return parent;
}

vector<int> WeightedGraph::DijkstraShortestPath(int startNode) {
    // Initialize distances from start node to all nodes as infinity
    vector<int> distances(numNodes, numeric_limits<int>::max());
    // Initialize start node distance as 0
    distances[startNode] = 0;

    // Priority queue to keep track of nodes with shortest distance
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    // Add start node to the queue
    pq.push(make_pair(0, startNode));

    while (!pq.empty()) {
        // Get the node with the shortest distance from the queue
        int currNode = pq.top().second;
        pq.pop();

        // Iterate over neighbors of the current node
        for (auto neighbor : adjList[currNode]) {
            int adjNode = neighbor.first;
            int edgeWeight = neighbor.second;
            // If a shorter path to the adjacent node is found through the current node
            if (distances[currNode] + edgeWeight < distances[adjNode]) {
                // Update distance to the adjacent node
                distances[adjNode] = distances[currNode] + edgeWeight;
                // Add the adjacent node to the queue
                pq.push(make_pair(distances[adjNode], adjNode));
            }
        }
    }

    // Return the distances from the start node to all nodes
    return distances;
}





AStarNode::AStarNode(int id, int gScore, int fScore)
    : id(id), gScore(gScore), fScore(fScore) {}

bool AStarNode::operator<(const AStarNode &other) const {
    return fScore > other.fScore;
}



vector<int> WeightedGraph::AStarSearch(int start, int goal, unordered_map<int, int> &heuristics){
    // Check if the start and goal nodes exist in the graph
    if (nodes.find(start) == nodes.end() || nodes.find(goal) == nodes.end()) {
        cerr << "Error: start or goal node not found in graph." << endl;
        return {};
    }

    // Initialize priority queue and closed set
    priority_queue<AStarNode> pq;
    unordered_set<int> closedSet;

    // Initialize the start node
    pq.push(AStarNode(start, 0, heuristics[start]));
    unordered_map<int, int> gScores = {{start, 0}};

    // Keep track of the parent node of each node in the optimal path
    unordered_map<int, int> parents;

    while (!pq.empty()) {
        // Get the node with the lowest f(n) score
        AStarNode curr = pq.top();
        pq.pop();

        // Check if we have reached the goal
        if (curr.id == goal) {
            vector<int> path = {curr.id};
            while (parents.find(path.front()) != parents.end()) {
                path.insert(path.begin(), parents[path.front()]);
            }
            return path;
        }

        // Add the current node to the closed set
        closedSet.insert(curr.id);

        // Explore neighbors of the current node
        for (auto &edge : adjList[curr.id]) {
            int neighbor = edge.first;
            if (closedSet.find(neighbor) != closedSet.end()) {
                continue; // Ignore neighbors that have already been evaluated
            }

            int tentativeGScore = gScores[curr.id] + edge.second;
            if (gScores.find(neighbor) == gScores.end() || tentativeGScore < gScores[neighbor]) {
                // This path to the neighbor is better than any previous one. Record it.
                parents[neighbor] = curr.id;
                gScores[neighbor] = tentativeGScore;
                int fScore = tentativeGScore + heuristics[neighbor];
                pq.push(AStarNode(neighbor, tentativeGScore, fScore));
            }
        }
    }

    cerr << "Error: no path found from start to goal." << endl;
    return {};
}

// pair<int, vector<vector<int>>> WeightedGraph::FordFulkerson(int source, int sink) {
//     cout<<"1111";
//     return make_pair(0, vector<vector<int>>{});
//     int maxFlow = 0;
//     vector<int> parent(numNodes, -1);
//     // create residual graph and initialize it
//     vector<vector<int>> residualGraph(numNodes, vector<int>(numNodes, 0));
//     for (int i = 0; i < numNodes; i++) {
//         for (auto neighbor : adjList[i]) {
//             residualGraph[i][neighbor.first] += neighbor.second;
//         }
//     }
//     cout<<111;
//     // repeatedly find augmenting paths and update the flow until no more augmenting paths exist
//     while (true) {
//         // find an augmenting path using DFS
//         vector<bool> visited(numNodes, false);
//         int flow = dfsFordFulkerson(source, sink, INT_MAX, parent, residualGraph);
//         // if no more augmenting paths exist, break
//         if (flow == 0) {
//             break;
//         }
//         // update the flow and residual graph
//         maxFlow += flow;
//         for (int v = sink; v != source; v = parent[v]) {
//             int u = parent[v];
//             residualGraph[u][v] -= flow;
//             residualGraph[v][u] += flow;
//         }
//         break;
//     }

//     return make_pair(maxFlow, residualGraph);
// }

// int WeightedGraph::dfsFordFulkerson(int u, int t, int flow, vector<int>& parent, vector<vector<int>>& residualGraph) {
//     parent[u] = t; // set the parent of u to t

//     int pathFlow = 0;
//     cout<<8;
//     // Base case: if we reach the sink node, return the current flow
//     if (u == t) {
//         cout<<2;

//         return flow;
//     }

//     // For each neighbor v of u in the residual graph
//     for (int v = 0; v < residualGraph.size(); v++) {
//         if (residualGraph[u][v] > 0 && parent[v] == -1) {
//             // Compute the minimum flow in the path
//             int minPathFlow = dfsFordFulkerson(v, t, min(flow, residualGraph[u][v]), parent, residualGraph);
//             if (minPathFlow > 0) {
//                 // Update the residual capacities of the edges
//                 residualGraph[u][v] -= minPathFlow;
//                 residualGraph[v][u] += minPathFlow;

//                 // Update the path flow
//                 pathFlow += minPathFlow;

//                 // Update the flow variable
//                 flow -= minPathFlow;

//                 if (flow <= 0) {
//                     break;
//                 }
//             }
//         }
//     }
//     return pathFlow;
// }
