#include "net.h"
#include "space.h"


namespace rt {

int Net::_getNodeIdx(const T3 &cord) {
  int ret;
  if (!_occupy(cord)) {
    ret = numNodes ++;
    nodes.insert({ret, Node(cord[0], cord[1], cord[2], ret)});
    occupied.insert({cord, ret});
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

void Net::constructGraph() {

// from Route construct a graph topo
// mark Node.isPin 

  for (auto route : routes) {
    T3 cord1{route.sRowIdx, route.sColIdx, route.sLayIdx};
    T3 cord2{route.eRowIdx, route.eColIdx, route.eLayIdx};
    int id1{_getNodeIdx(cord1)};
    int id2{_getNodeIdx(cord2)};

    auto node_ptr1 = nodes.find(id1); 
    auto node_ptr2 = nodes.find(id2);
    
    node_ptr1->second.deg += 1;
    node_ptr2->second.deg += 1;
    
    int edgeIdx = _getEdgeIdx(id1, id2);

    node_ptr1->second.addLink(edgeIdx);
    node_ptr2->second.addLink(edgeIdx);
  }

  for (auto pin : _getNetPins()) {
    nodes[occupied[pin]].isPin = 1;
  }
}

void Net::optimizeGraph() {
  // To be implemented
  _graph2Tree();
  _cutLeaves();
}

} // namespace rt
