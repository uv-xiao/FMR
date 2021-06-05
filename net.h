#pragma once

#include "types.h"
#include "parser.h"

#include <vector>
#include <array>
#include <string>
#include <map>


namespace rt {

struct Node {
  int x, y, z;
  int id;
  bool isPin;
  int deg;
  std::set<int> link; // store edges from the node
  Node() {}
  Node(int x, int y, int z, int id)
    :x(x), y(y), z(z), id(id) {}
  void addLink(int edgeIdx) {
    if (link.find(edgeIdx) != link.end())
      link.insert(edgeIdx);
  }
};

struct Edge {
  // id1 < id2
  int id1, id2;
  Edge() {}
  Edge(int id1, int id2)
    :id1(id1), id2(id2) {}
};


class Net {
private:

  // A Net must be in a space;
  Space &space;
  
  // Information about the net
  db::Net basics;
  
  // Edges on 3D space
  std::vector<db::Route> routes;
  
  // map from idx to node
  int numNodes = 0;
  std::map<int, Node> nodes;

  // map from edgeID to edge 
  int numEdges = 0;
  std::map<int, Edge> edges;

  // map from grid cord to idx
  std::map<T3, int> occupied;
  
  // map from <idx, idx> to edgeID
  std::map<std::pair<int, int>, int> edgeIdx;

  bool _occupy(const T3 &b) { 
    return occupied.find(b) != occupied.end(); 
  }
  int _getNodeIdx(const T3 &cord);
  int _getEdgeIdx(int a, int b); 
  std::vector<T3> _getNetPins();

  void _graph2Tree();
  void _cutLeaves();

public:

  Net(Space &space);

  void constructGraph();
  // optimize: Graph -> Tree -> Core Tree
  void optimizeGraph();

  void setBasics(const db::Net &net) {
    this->basics = net;
  }
  void setRoutes(const std::vector<db::Route> &vec) {
    this->routes = vec;
  }

  const std::map<T3, int> & getOccupiedCells() {
    return occupied;
  }
};

} // namespace rt