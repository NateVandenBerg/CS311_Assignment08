#include "graph.h"
using namespace std;

// @brief Construct a Graph with the given number of vertices.
// @param nV The number of vertices in the graph.
Graph::Graph(int nV) {
    numVerts = nV;
    vertices = vector<Vertex>(numVerts);
    adjList = vector<vector<Edge>>(numVerts);
}

// @brief destructor
Graph::~Graph() {
    vertices.clear();
    adjList.clear();
}

// @brief add a vertex to the graph
void Graph::addVertex(Vertex v)
{
    //Add a vertex to the vertex vector
    vertices.push_back(v);
    //new blank edge list
    vector<Edge> newEdgeList;
    //create new vector of type edge in the adjList vector
    adjList.push_back(newEdgeList);
    numVerts++;
}

// @brief add a directed edge v1->v2 to the graph
void Graph::addDirectedEdge(int v1, int v2, float weight)
{
    //check if edge has already been added
    //Traverse the vector
    for (auto it : adjList[v1]) {
        //check if edge already exists
        if (v1 == it.from_vertex &&
            v2 == it.to_vertex &&
            weight == it.weight) {
            return;//do nothing if edge already exists
        }
    }

    //Create the Edge
    Edge e = Edge(v1, v2, weight);
    //push edge onto the adjList vector
    adjList[v1].push_back(e);
}

// @brief add an undirected edge to the graph
void Graph::addUndirectedEdge(int v1, int v2, float weight) {
    //Add edge v1->v2
    addDirectedEdge(v1, v2, weight);
    //Add edge v2->v1
    addDirectedEdge(v2, v1, weight);
}

// @brief the number of outgoing edges from a vertex
int Graph::outDegree(int v) {
    //return the number of edges coming from the vector
    return adjList[v].size();
}

// @brief depth first search from vertex v
vector<int> Graph::DepthFirstSearch(int v) {
    //create the traveral list
    vector<int> DFStraversal;
    //create a list for vertices that have been visited
    vector<bool> visited(numVerts, false);
    //create a stack of vertices that need to be visited
    std::stack<int> list;
    //add start vertex to list
    list.push(v);
    //run DFS recursively
    recursiveDFS(DFStraversal, visited, list);

    return DFStraversal;
}

void Graph::recursiveDFS(vector<int> &DFStraversal, vector<bool> &visited, std::stack<int> &list) {
    //as long as the stack is not empty
    while (!list.empty()) {
        //get the top vertex from the stack
        int curr = list.top();
        //remove the vertex from the stack
        list.pop();
        //set the vertex as visited
        visited[curr] = true;
        //add the vertex as part of the traversal
        DFStraversal.push_back(curr);
        //traverse the adjacent edge list
        for (auto it : adjList[curr]) {
            //if that vertex has not been visited yet
            if (!visited[it.to_vertex]) {
                //add the vertex to the stack
                list.push(it.to_vertex);
                //recursively call DFS at this vertex
                recursiveDFS(DFStraversal, visited, list);
            }
        }
    }
}

// @brief breadth first search from a vertex
vector<int> Graph::BreadthFirstSearch(int v) {
    //List of vertices traversed
    vector<int> BFStraversal;
    //used to check if vetex has been visited already
    vector<bool> visited(numVerts, false);
    //create queue of vertices to be visited
    queue<int> frontierQueue;
    //mark starting vertex as visited
    visited[v] = true;
    //add starting vertex to queue
    frontierQueue.push(v);

    while (!frontierQueue.empty()) {
        //get next vertex from the queue
        int curr = frontierQueue.front();
        //remove it from the queue
        frontierQueue.pop();
        //add the visited vertex to the traversal list
        BFStraversal.push_back(curr);
        //at the current vertex get the adjacent vertices
        for (auto it : adjList[curr]) {
            //if this adjacent vertex has not been visited
            if (!visited[it.to_vertex]) {
                //update the vertex as visited
                visited[it.to_vertex] = true;
                //add the vertex to the queue
                frontierQueue.push(it.to_vertex);
            }
        }
    } 
    return BFStraversal;
}

/**
 * @brief Check if the graph has cycles
 * @return true if the graph has cycles
 */
bool Graph::checkCycle() {
    //traverse vertices vector
    for (auto v : vertices) {
        //create a list for vertices that have been visited
        vector<bool> visited(numVerts, false);
        //if the vertex has not been visited yet
        if (!visited[v.id])
            //check of the graph at the vertex has a loop
            if (checkCycleRecursive(v.id, visited, -1))
                //return true if it does
                return true;
    }
    //else return false
    return false;
}

bool Graph::checkCycleRecursive(int v, vector<bool>& visited, int parent) {
    //set the vertex as visited
    visited[v] = true;
    //traverse the adjacent edge list
    for (auto it : adjList[v]) {
        //if the adjacent vertex has not been visited yet
        if (!visited[it.to_vertex]) {
            //check if the graph of the adjacnet vertex contains a loop
            if (checkCycleRecursive(it.to_vertex, visited, v))
                //if it does return true
                return true;
        }
        //otherwise, if the adjacent vertex in not the parent vertex
        else if (it.to_vertex != parent)
            //return true
            return true;
    }
    //else return false
    return false;
}

// @brief print the graph
void Graph::printGraph()
{
    cout << "Graph:" << endl;
    for (int i = 0; i < numVerts; i++)
    {
        cout << i << ": ";
        for(auto j = adjList[i].begin(); j != adjList[i].end(); ++j)
        {
            cout << (*j).to_vertex << " " ;
        }
        cout << endl;
    }
    cout << endl;
}
