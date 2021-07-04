#pragma once

#include "types.h"
#include "parser.h"
#include "config.h"
#include "flute/flute.h"


#include <vector>
#include <array>
#include <string>
#include <map>


extern "C" {
Tree flute(int d, DTYPE x[], DTYPE y[], int acc);
}

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
  void removeLink(int edgeIdx) {
    link.erase(edgeIdx);
  }
  operator T3 () const {
    return T3{x, y, z};
  }
};


struct Edge {
  // id1 < id2
  int id1, id2;
  Edge() {}
  Edge(int id1, int id2)
    :id1(id1), id2(id2) {}
  operator T2() const {
    return T2{id1, id2};
  }
};


class Net {
private:

  const double violationCost = 1e7;
  
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


  void _removeNode(int x);
  void _removeEdge(int edgeID);
  int _addNode(const T3 &cord);
  int _addEdge(int a, int b, bool initial);
  int _getNodeIdx(const T3 &cord);
  int _getEdgeIdx(int a, int b, bool initial); 
  std::vector<T3> _getNetPins();

  int _getCenter();

  void _optimize();
  void _checkLegality();

  double _estCost(const T3 a);
  bool _simpleRouteDFS(const T3 a, const T3 b, std::vector<T3> &passed,
                       const int lastDir = -1);
  void _simpleRoute2Pins(const T3 a, const T3 b, const int lastDir = -1);
  void _simpleRoute(cf::Config &config);
  
  void _aStarRoute2Pins(const T3 &a, const T3 &b);
  void _aStarRoute(cf::Config &config);
  
  void _removeEdgeFromGrid(T3 from, T3 to);
  void _addEdge2Grid(T3 from, T3 to);
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
  
  void cleanAll();
  void modifyCells();
  void route(cf::Config &config);
  void reroute(cf::Config &config);
};

} // namespace rt