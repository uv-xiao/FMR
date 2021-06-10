#include "net.h"
#include "space.h"
#include <assert.h>
#include <deque>
#include <algorithm> 
#include <climits>

namespace rt {

Net::Net(Space &space) : space(space){}

int Net::_getNodeIdx(const T3 &cord) {
  int ret;
  if (!_occupy(cord)) {
    ret = numNodes ++;
    nodes.insert({ret, Node(cord[0], cord[1], cord[2], ret)});
    occupied.insert({cord, ret});
    space._addDemandOnGrid(cord, 1);
  } 
  else
    ret = occupied[cord];
  return ret;
}

int Net::_getEdgeIdx(int a, int b) {
  if (a == b) {
    throw std::runtime_error("self-edge");
  }
  if (a > b) std::swap(a, b);
  int ret;
  if (edgeIdx.find({a,b}) == edgeIdx.end()) {
    ret = numEdges ++;
    edges.insert({ret, Edge(a, b)});
    edgeIdx.insert({{a, b}, ret});
  }
  else 
    ret = edgeIdx[{a, b}];
  return ret;
}

std::vector<T3> Net::_getNetPins() {
  std::vector<T3> ret;
  for (auto pinName : basics.pins) {
    auto cell = space.cellInss[pinName[0]];
    int x{cell.rowIdx}, y{cell.colIdx};
    auto mc = space.chip.masterCells[cell.mcName];
    db::Pin pinBasic = mc.pins[pinName[1]];
    std::string layerName = pinBasic.layer;
    int z{space.chip.layerName2Idx[layerName]};
    ret.push_back(T3{x, y, z});
  }
  return ret;
}

int Net::_addNode(const T3 &cord) {
  return _getNodeIdx(cord);
}
int Net::_addEdge(int a, int b) {
  return _getEdgeIdx(a, b);
}

void Net::_removeNode(int x) {
  assert(nodes.find(x) != nodes.end() && "must remove existing node");
  space._addDemandOnGrid(nodes[x], -1);
  occupied.erase(nodes[x]);
  nodes.erase(x);
}

void Net::_removeEdge(int edgeID) {
  edgeIdx.erase(edges[edgeID]);
  edges.erase(edgeID);
}

int Net::_getCenter() {
  std::vector<int> xs, ys;
  auto pins = _getNetPins();
  for (auto pin : pins) {
    xs.push_back(pin[0]);
    ys.push_back(pin[1]);
  }
  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  int mid_x = (xs.size() % 2 == 0) ? 
                (xs[xs.size()/2] + xs[xs.size()/2+1]) / 2
              : xs[xs.size()/2];
  int mid_y = (ys.size() % 2 == 0) ?
                (ys[ys.size()/2] + ys[ys.size()/2+1]) / 2
              : ys[ys.size()/2];
  
  int x, min_dis = INT_MAX, i = 0;
  for (auto pin : pins) {
    if (std::abs(pin[0] - mid_x) + std::abs(pin[1] - mid_y)
        < min_dis) {
      min_dis = std::abs(pin[0] - mid_x) + std::abs(pin[1] - mid_y);
      x = i; 
    }
    i += 1;
  }
  return occupied[pins[i]];
}

void Net::_checkLegality() {

}

void Net::_optimize() {
  int id = _getCenter();

  std::map<int, int> father;
  std::map<int, int> visited;
  visited.insert({id, 0});
  std::deque<int> q;
  assert(nodes.find(id) == nodes.end() && "center is in nodes");
  q.push_back(id);

  while(!q.empty()) {
    int x = q.front();
    q.pop_front();
    Node &node_x = nodes[x];
    for (auto link : node_x.link) {
      assert(edges.find(link) != edges.end() && "edge is in edges");
      Edge &edge = edges[link];
      int y = edge.id1 + edge.id2 - x;
      assert(nodes.find(y) != nodes.end() && "y is in nodes");
      Node &node_y = nodes[y];
      if (visited.find(y) != visited.end()) {
        // find a cycle, just break it
        node_x.removeLink(id);
        node_y.removeLink(id);
        _removeEdge(id);
      }
      else {
        visited[x] += 1;
        visited.insert({y, 0});
        father.insert({y, x});
        if (node_y.isPin)
          q.push_front(y);
        else
          q.push_back(y);
      }
    }
  }

  for (auto pr : visited) 
    if (pr.second == 0 && !nodes[pr.first].isPin)
      // i-node leaves
      q.push_back(pr.first);
  
  while (!q.empty()) {
    int x = q.front();
    q.pop_front();
    Node &node_x = nodes[x];
    assert(father.find(x) != father.end() && "x's father not exist");
    int y = father[x];
    visited[y] -= 1;
    if (visited[y] == 0)
      q.push_back(y);
    
    _removeEdge(_getEdgeIdx(x, y));
    _removeNode(x);
  }

  _checkLegality();
}


void Net::constructGraph() {

// from Route construct a graph topo
// mark Node.isPin 

  for (auto route : routes) {
    T3 cord1{route.sRowIdx, route.sColIdx, route.sLayIdx};
    T3 cord2{route.eRowIdx, route.eColIdx, route.eLayIdx};
    int id1{_addNode(cord1)};
    int id2{_addNode(cord2)};

    auto node_ptr1 = nodes.find(id1); 
    auto node_ptr2 = nodes.find(id2);
    
    node_ptr1->second.deg += 1;
    node_ptr2->second.deg += 1;
    
    int edgeIdx = _addEdge(id1, id2);

    node_ptr1->second.addLink(edgeIdx);
    node_ptr2->second.addLink(edgeIdx);
  }

  for (auto pin : _getNetPins()) {
    nodes[occupied[pin]].isPin = 1;
  }
}

void Net::optimizeGraph() {
  _optimize();
}

void Net::cleanAll() {
  std::vector<int> toErase;
  for (auto node : nodes) {
    toErase.push_back(node.first);
  }
  for (auto i : toErase)
    _removeNode(i);
  numNodes = 0;

  edges.clear();
  edgeIdx.clear();
  numEdges = 0;
}

void Net::modifyCells() {
  cleanAll();
  routes.clear();
}

void Net::_simpleRoute(cf::Config &config) {

}

void Net::_aStarRoute(cf::Config &config) {

}

void Net::route(cf::Config &config) {
  // std::vector<T3> pins(_getNetPins());
  if (config.getMode())
    _aStarRoute(config);
  else
    _simpleRoute(config);
}

void Net::reroute(cf::Config &config) {
  modifyCells();
  route(config);
}

} // namespace rt
