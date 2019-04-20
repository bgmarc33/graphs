#include <iostream>
#include "Graph.h"

int main() {
    ALGraph<std::string> g;
    g.addVertex("NY");
    g.addVertex("MA");
    g.addVertex("CA");

    g.addEdge("MA", "CA", 1300);
    g.addEdge("NY", "CA", 1000);

    std::cout << g.isConnected() << std::endl;

    AMGraph g1(3);
    return 0;
}