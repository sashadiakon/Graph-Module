#include "weighted_graph.h"
using namespace std;

void WeightedGraph::readFromFile(const std::string& fileName) {
    std::ifstream inFile(fileName);
    if (!inFile) {
        std::cerr << "Error: Failed to open file " << fileName << std::endl;
        return;
    }

    int numNodes, numEdges;
    inFile >> numNodes >> numEdges;

    for (int i = 0; i < numNodes; ++i) {
        addNode(i);
    }

    for (int i = 0; i < numEdges; ++i) {
        int source, dest, weight;
        inFile >> source >> dest >> weight;
        addEdge(source, dest, weight);
    }

    inFile.close();
}

void WeightedGraph::addNode(int node) {
    adjList[node] = std::vector<std::pair<int, int>>();
}

void WeightedGraph::addEdge(int node1, int node2, int weight) {
    adjList[node1].push_back({node2, weight});
    adjList[node2].push_back({node1, weight});
}

void WeightedGraph::print() {
    for (const auto &node : adjList) {
        std::cout << node.first << ": ";
        for (const auto &neighbor : node.second) {
            std::cout << neighbor.first << "(" << neighbor.second << ") ";
        }
        std::cout << std::endl;
    }
}

vector<int> WeightedGraph::BFS(int start) {
    std::unordered_map<int, bool> visited;
    vector<int> path;
    for (const auto &node : adjList) {
        visited[node.first] = false;
    }

    std::queue<int> q;
    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        int node = q.front();
        q.pop();

        std::cout << node << " ";
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
    std::stack<int> stack;
    std::unordered_map<int, bool> visited;
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
        std::cout << node << " ";
        path.push_back(node);
        for (auto &neighbor : adjList[node]) {
            stack.push(neighbor.first);
        }
        }
    }
    std::cout << std::endl;
    return path;
}

void WeightedGraph::PrimMST() {
    numNodes = 3;
    std::vector<int> key(numNodes, INT_MAX);
    std::vector<int> parent(numNodes, -1);
    std::vector<bool> inMST(numNodes, false);
    key[0] = 0;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                        std::greater<std::pair<int, int>>>pq;
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
        std::cout << parent[i] << " - " << i << std::endl;
    }
}

std::vector<int> WeightedGraph::DijkstraShortestPath(int startNode) {
    // Initialize distances from start node to all nodes as infinity
    std::vector<int> distances(numNodes, std::numeric_limits<int>::max());
    // Initialize start node distance as 0
    distances[startNode] = 0;

    // Priority queue to keep track of nodes with shortest distance
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;
    // Add start node to the queue
    pq.push(std::make_pair(0, startNode));

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
                pq.push(std::make_pair(distances[adjNode], adjNode));
            }
        }
    }

    // Return the distances from the start node to all nodes
    return distances;
}





class AStarNode {
public:
    int id;
    int fScore; // f(n) = g(n) + h(n)
    int gScore; // g(n) - cost to reach this node
    AStarNode(int id, int gScore, int fScore) : id(id), gScore(gScore), fScore(fScore) {}
    bool operator<(const AStarNode &other) const {
        return fScore > other.fScore;
    }
};


std::vector<int> WeightedGraph::AStarSearch(int start, int goal, std::unordered_map<int, int> &heuristics){
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
