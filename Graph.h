//
// Created by Bryan Giordano on 4/20/19.
//

#ifndef GRAPHS_GRAPH_H
#define GRAPHS_GRAPH_H

#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <unordered_map>

/*
 *  Edge struct for neighboring vertices
 */

struct Edge {
    int destination;
    int weight;

    Edge(int d, int w=0) {
        this->destination = d;
        this->weight = w;
    }
};

/*
 *  T represents the vertex values
 */

template <class T>
class Graph {
    private:
        std::unordered_map<T, int> vertexList;
        std::unordered_map<int, T> vertexDetailList;
        std::vector<std::vector<Edge>> vertices;
    public:
        Graph() {}

        Graph(T* vertices, int size) {
            for (int i = 0; i < size; i++) {
                this->vertexList[vertices[i]] = i;
                this->vertexDetailList[i] = vertices[i];
                this->vertices.push_back(std::vector<Edge>());
            }
        }

        int vertexToIndex(T vertex) {
            if (this->vertexList.find(vertex) != this->vertexList.end()) {
                return this->vertexList[vertex];
            }

            return -1;
        }

        T indexToVertex(int index) {
            if (index > this->vertexDetailList.size()) {
                return NULL;
            }

            return this->vertexDetailList[index];
        }

        void addVertex(T vertex) {
            int vertexIndex = this->vertexList.size();
            this->vertexList[vertex] = vertexIndex;
            this->vertexDetailList[vertexIndex] = vertex;
            this->vertices.push_back(std::vector<Edge>());
        }

        void addEdge(T start, T destination, int w = 0) {
            int startIndex = this->vertexToIndex(start);
            int endIndex = this->vertexToIndex(destination);

            if (startIndex != -1 && endIndex != -1) {
                Edge edge(startIndex, w);
                Edge edge2(endIndex, w);

                this->vertices[startIndex].push_back(edge2);
                this->vertices[endIndex].push_back(edge);
            }
        }

        /*
         *  Depth first search
         */

        void DFS(T vertex) {
            if (this->vertexList.size() > 0) {
                int vertexIndex = this->vertexToIndex(vertex);
                if (vertexIndex != -1) {
                    bool visited[this->vertexList.size()];
                    for (int i = 0; i < this->vertexList.size(); i++) {
                        visited[i] = false;
                    }

                    this->_DFSRec(0, visited);
                }
            }
        }

        /*
         *  DFS Recursive Approach
         */

        void _DFSRec(int vertex, bool visited[]) {
            // std::cout << "Vertex: " << this->indexToVertex(vertex) << std::endl;
            visited[vertex] = true;
            for (int i = 0; i < this->vertices[vertex].size(); i++) {
                Edge e = this->vertices[vertex][i];
                if (!visited[e.destination]) {
                    visited[e.destination] = true;
                    _DFSRec(e.destination, visited);
                }
            }
        }

        /*
         *  DFS Iterative Approach
         */

        void _DFSIter(int vertex, bool visited[]) {
            std::stack<int> vertexStack;
            vertexStack.push(vertex);
            visited[vertex] = true;

            while (!vertexStack.empty()) {
                int currentVertex = vertexStack.top();
                vertexStack.pop();

                // std::cout << "Vertex: " << this->indexToVertex(currentVertex) << std::endl;

                for (int i = 0; i < this->vertices[currentVertex].size(); i++) {
                    Edge e = this->vertices[currentVertex][i];
                    if (!visited[e.destination]) {
                        vertexStack.push(e.destination);
                        visited[e.destination] = true;
                    }
                }
            }
        }

        /*
         *  If all vertices share a path
         */

        bool isConnected() {
            bool visited[this->vertexList.size()];
            for (int i = 0; i < this->vertexList.size(); i++) {
                visited[i] = false;
            }

            this->_DFSIter(0, visited);
            for (int i = 0; i < this->vertexList.size(); i++) {
                if (!visited[i]) {
                    return false;
                }
            }

            return true;
        }

        /*
         *  Breadth first search
         */

        void BFS(T vertex) {
            if (this->vertexList.size() > 0) {
                int vertexIndex = this->vertexToIndex(vertex);
                if (vertexIndex != -1) {
                    bool visited[this->vertexList.size()];
                    for (int i = 0; i < this->vertexList.size(); i++) {
                        visited[i] = false;
                    }

                    this->_BFS(0, visited);
                }
            }
        }

        void _BFS(int vertex, bool visited[]) {
            std::queue<int> vertexQueue;
            vertexQueue.push(vertex);
            visited[vertex] = true;

            while (!vertexQueue.empty()) {
                int currentVertex = vertexQueue.front();
                vertexQueue.pop();

                // std::cout << "Vertex: " << this->indexToVertex(currentVertex) << std::endl;

                for (int i = 0; i < this->vertices[currentVertex].size(); i++) {
                    Edge e = this->vertices[currentVertex][i];
                    if (!visited[e.destination]) {
                        vertexQueue.push(e.destination);
                        visited[e.destination] = true;
                    }
                }
            }
        }

};


#endif //GRAPHS_GRAPH_H
