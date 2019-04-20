#include <iostream>
#include "Graph.h"

int main() {
    Graph<std::string> g;
    g.addVertex("NY");
    g.addVertex("MA");
    g.addVertex("CA");

    g.addEdge("NY", "CA", 1000);

    g.DFS("NY");
    return 0;
}