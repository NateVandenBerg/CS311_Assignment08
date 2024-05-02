//CS311 graph.h
//INSTRUCTION:
//undirected graph class - header file template
//You should always add comments to each function to describe its PURPOSE and PARAMETERS
#pragma once
#include <vector>
#include <list>
#include <queue>
#include <iostream>
#include <stack>
using namespace std;

/**
 * @brief Vertex class
 * 
 */
class Vertex {
public:
    // The index of the vertex. starting from 0
    int id;
    // The name of the vertex. Additional attributes can be added
    string name;

    Vertex(int id = 0, string name="") {
        this->id = id;
        this->name = name;
    }
};

/**
 * @brief Edge class. It represents an edge from one vertex to another
 * 
 */
struct Edge {
    int from_vertex; // The index of the vertex where the edge starts
    int to_vertex; // index of the vertex where the edge ends.
    float weight; // The weight of the edge. Additional attributes can be added

    Edge(int from_vertex = 0, int to_vertex = 0, float weight = 0) {
        this->from_vertex = from_vertex;
        this->to_vertex = to_vertex;
        this->weight = weight;
    }
};

/**
 * @brief Graph class
 *
 */
class Graph
{
public:
    int numVerts;    // No. of vertices
    vector<Vertex> vertices; // The list of vertices
    vector<vector<Edge>> adjList; // The adjacency list

    // Default constructor. Create an empty graph
    Graph() {
        numVerts = 0;
    }

    // Constructor. Create a graph with n vertices
    Graph(int n);

    // Destructor
    ~Graph();

    /**
     * @brief Add a vertex to the graph
     * @param v The vertex to be added
     */
    void addVertex(Vertex v);

    /**
     * @brief Add a directed edge from v1 to v2 to the graph
     *
     * @param v1 The index of the vertex where the edge starts
     * @param v2 The index of the vertex where the edge ends
     * @param weight The weight of the edge
     */
    void addDirectedEdge(int v1, int v2, float weight = 1.0f);

    /**
     * @brief Add an undirected edge to the graph. An undirected edge is represented by two directed edges.
     * @param v1 The index of the first vertex
     * @param v2 The index of the second vertex
     */
     void addUndirectedEdge(int v1, int v2, float weight = 1.0f);

    /**
     * @brief the number of outgoing edges from vertex v
     * @param v The index of the vertex
     * @return The number of outgoing edges from the vertex
     */
    int outDegree(int v);

    /**
     * @brief Depth first search
     *
     * @param v The index of the vertex where the search starts
     * @return The list of vertices in the order they are visited
     */
    vector<int> DepthFirstSearch(int v);
	
	void recursiveDFS(vector<int> &DFStraversal, vector<bool> &visited, stack<int> &list);

    /**
     * @brief Breadth first search
     *
     * @param v The index of the vertex where the search starts
     * @return The list of vertices in the order they are visited
     */
    vector<int> BreadthFirstSearch(int v);

    /**
     * @brief Check if the undirected graph contains cycles
     * @return true if the graph has cycles
     */
    bool checkCycle();
	
	bool checkCycleRecursive(int u, vector<bool>& visited, int parent);

    /**
      * @brief Print the graph
      */
    void printGraph();

};