#include "net.h"

#include <assert.h>

#include <algorithm>
#include <climits>
#include <deque>
#include <list>

#include "space.h"

namespace rt {

Net::Net(Space &space) : space(space) {}

int Net::_getNodeIdx(const T3 &cord) {
  int ret;
  if (!_occupy(cord)) {
    ret = numNodes++;
    nodes.insert({ret, Node(cord[0], cord[1], cord[2], ret)});
    occupied.insert({cord, ret});
    if (cord == T3{17, 84, 7}) {
      FILE *f = fopen("debug", "a");
      fprintf(f, "before op, demand = %d\n", space._getDemandOnGrid(cord));
      fprintf(f, "net %s add node demand on (17,84,7)\n",
              basics.netName.c_str());
      fclose(f);
    }
    space._addNet2Grid(cord, basics.netName);
    if (cord == T3{17, 84, 7}) {
      FILE *f = fopen("debug", "a");
      fprintf(f, "after op, demand = %d\n", space._getDemandOnGrid(cord));
      fclose(f);
    }
  } else
    ret = occupied[cord];
  return ret;
}

int Net::_getEdgeIdx(int a, int b, bool initial = false) {
  if (a == b) {
    throw std::runtime_error("self-edge");
  }
  if (a > b) std::swap(a, b);
  int ret;
  if (edgeIdx.find({a, b}) == edgeIdx.end()) {
    ret = numEdges++;
    if (initial) _addEdge2Grid(T3(nodes[a]), T3(nodes[b]));
    edges.insert({ret, Edge(a, b)});
    edgeIdx.insert({{a, b}, ret});
  } else
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

int Net::_addNode(const T3 &cord) { return _getNodeIdx(cord); }
int Net::_addEdge(int a, int b, bool initial = false) {
  // std::cout << "add edge!!!" << std::endl;
  return _getEdgeIdx(a, b, initial);
}

void Net::_removeEdgeFromGrid(T3 from, T3 to) {
  int neq = 0, direct;
  for (int i = 0; i < 3; i++)
    if (from[i] != to[i]) {
      neq += 1;
      direct = i;
    }
  std::cerr << "remove edge " << from[0] << "," << from[1] << "," << from[2]
            << "->" << to[0] << "," << to[1] << "," << to[2] << std::endl;
  if (neq == 1) {
    if (from[direct] > to[direct]) std::swap(from, to);
    T3 tmp = from;
    for (int i = from[direct] + 1; i < to[direct]; i++) {
      tmp[direct] = i;
      std::cerr << "remove edge node" << tmp[0] << tmp[1] << tmp[2]
                << std::endl;
      if (tmp == T3{17,84,7}) {
        FILE *f = fopen("debug", "a");
        fprintf(f, "before op, demand = %d\n", space._getDemandOnGrid(tmp));
        fprintf(f, "net %s remove edge demand on (17,84,7)\n",
                basics.netName.c_str());
        fclose(f);
      }
      space._removeNetFromGrid(tmp, basics.netName);
      if (tmp == T3{17,84,7}) {
        FILE *f = fopen("debug", "a");
        fprintf(f, "after op, demand = %d\n", space._getDemandOnGrid(tmp));
        fclose(f);
      }
    }
  } else
    assert("must be in x/y/z direction");
}

void Net::_addEdge2Grid(T3 from, T3 to) {
  int neq = 0, direct;
  for (int i = 0; i < 3; i++)
    if (from[i] != to[i]) {
      neq += 1;
      direct = i;
    }
  if (neq == 1) {
    if (from[direct] > to[direct]) std::swap(from, to);
    T3 tmp = from;
    for (int i = from[direct] + 1; i < to[direct]; i++) {
      tmp[direct] = i;
      if (tmp == T3{17,84,7}) {
        FILE *f = fopen("debug", "a");
        fprintf(f, "before op, demand = %d\n", space._getDemandOnGrid(tmp));
        fprintf(f, "net %s add edge demand on (17,84,7)\n",
                basics.netName.c_str());
        fclose(f);
      }
      space._addNet2Grid(tmp, basics.netName);
      if (tmp == T3{17,84,7}) {
        FILE *f = fopen("debug", "a");
        fprintf(f, "after op, demand = %d\n", space._getDemandOnGrid(tmp));
        fclose(f);
      }
    }
  } else
    assert("must be in x/y/z direction");
}

void Net::_removeNode(int x) {
  assert(nodes.find(x) != nodes.end() && "must remove existing node");
  T3 cord = nodes[x];
  if (cord == T3{17,84,7}) {
    FILE *f = fopen("debug", "a");
    fprintf(f, "before op, demand = %d\n", space._getDemandOnGrid(cord));
    fprintf(f, "net %s remove node demand on (17,84,7)\n",
            basics.netName.c_str());
    fclose(f);
  }
  space._removeNetFromGrid(nodes[x], basics.netName);
  if (cord == T3{17,84,7}) {
    FILE *f = fopen("debug", "a");
    fprintf(f, "after op, demand = %d\n", space._getDemandOnGrid(cord));
    fclose(f);
  }
  occupied.erase(nodes[x]);
  nodes.erase(x);
}

void Net::_removeEdge(int edgeID) {
  auto &edge = edges[edgeID];
  _removeEdgeFromGrid(nodes[edge.id1], nodes[edge.id2]);
  edgeIdx.erase(edge);
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
  int mid_x = (xs.size() % 2 == 0)
                  ? (xs[xs.size() / 2] + xs[xs.size() / 2 + 1]) / 2
                  : xs[xs.size() / 2];
  int mid_y = (ys.size() % 2 == 0)
                  ? (ys[ys.size() / 2] + ys[ys.size() / 2 + 1]) / 2
                  : ys[ys.size() / 2];

  int x, min_dis = INT_MAX, i = 0;
  for (auto pin : pins) {
    if (std::abs(pin[0] - mid_x) + std::abs(pin[1] - mid_y) < min_dis) {
      min_dis = std::abs(pin[0] - mid_x) + std::abs(pin[1] - mid_y);
      x = i;
    }
    i += 1;
  }
  return occupied[pins[i]];
}

bool Net::_checkLegality() {
  bool legal = true;
  for (auto &edge : edges) {
    T3 v1 = nodes[edge.second.id1];
    legal = space._getSupplyOnGrid(v1) > space._getDemandOnGrid(v1);
    if (!legal) break;
    T3 v2 = nodes[edge.second.id2];
    legal = space._getSupplyOnGrid(v1) > space._getDemandOnGrid(v1);
    if (!legal) break;
  }
  return legal;
}

void Net::_optimize() {
  int id = _getCenter();

  std::map<int, int> father;
  std::map<int, int> visited;
  visited.insert({id, 0});
  std::deque<int> q;
  assert(nodes.find(id) == nodes.end() && "center is in nodes");
  q.push_back(id);

  while (!q.empty()) {
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
      } else {
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
    if (visited[y] == 0) q.push_back(y);

    _removeEdge(_getEdgeIdx(x, y));
    _removeNode(x);
  }

  _checkLegality();
}

void Net::constructGraph() {
  // from Route construct a graph topo
  // mark Node.isPin
  if (routes.size() > 0)
    for (auto route : routes) {
      T3 cord1{route.sRowIdx, route.sColIdx, route.sLayIdx};
      T3 cord2{route.eRowIdx, route.eColIdx, route.eLayIdx};
      int id1{_addNode(cord1)};
      int id2{_addNode(cord2)};

      auto node_ptr1 = nodes.find(id1);
      auto node_ptr2 = nodes.find(id2);

      node_ptr1->second.deg += 1;
      node_ptr2->second.deg += 1;
      int edgeIdx = _addEdge(id1, id2, 1);
      node_ptr1->second.addLink(edgeIdx);
      node_ptr2->second.addLink(edgeIdx);
    }
  else
    for (auto pin : _getNetPins()) _addNode(pin);
  for (auto pin : _getNetPins()) {
    nodes[occupied[pin]].isPin = 1;
  }
  // if (basics.netName == "N22")
  // exit(-1);
}

void Net::optimizeGraph() { _optimize(); }

void Net::cleanAll() {
  for (auto &edge : edges)
    _removeEdgeFromGrid(nodes[edge.second.id1], nodes[edge.second.id2]);
  edges.clear();
  edgeIdx.clear();
  numEdges = 0;
  std::set<T3> loctoErase;
  for (auto node : nodes) {
    loctoErase.insert(node.second);
  }
  for (auto loc : loctoErase) {
    if (loc == T3{17,84,7}) {
      FILE *f = fopen("debug", "a");
      fprintf(f, "before op, demand = %d\n", space._getDemandOnGrid(loc));
      fprintf(f, "net %s remove node demand on (17,84,7)\n",
              basics.netName.c_str());
      fclose(f);
    }
      space._removeNetFromGrid(loc, basics.netName);
    if (loc == T3{17,84,7}) {
      FILE *f = fopen("debug", "a");
      fprintf(f, "after op, demand = %d\n", space._getDemandOnGrid(loc));
      fclose(f);
    }
    occupied.erase(loc);
  }
  nodes.clear();
  numNodes = 0;
}

void Net::modifyCells() {
  cleanAll();
  routes.clear();
}

double Net::_estCost(const T3 a, const int &d) {
  // TODO: design better
  double ret = 0;
  auto congestionFunc = [](int supply, int demand) {
    return (double)supply / (supply - demand);
  };
  if (a[0] < boundingBox.first.first || a[0] > boundingBox.first.second ||
      a[1] < boundingBox.second.first || a[1] > boundingBox.second.second)
    return violationCost;
  if (!_occupy(a)) {
    ret += basics.weight * space.chip.layers[a[2]].powerFactor;
    int demand = space._getDemandOnGrid(a);
    int supply = space._getSupplyOnGrid(a);
    if (demand >= supply)
      ret += violationCost * 1.5;
    else {
      ret += congestionFunc(supply, demand);
    }
  }
  if (d < 4) ret = ret * 0.75;
  if (d == 4) ret = ret * 0.9;
  return ret;
}

bool Net::_simpleRouteDFS(const T3 a, const T3 b, std::vector<T3> &passed,
                          std::set<T3> &reached, const int lastDir) {
  // std::cerr << "Route: (" << std::get<0>(a) << "," << std::get<1>(a) << ","
  //           << std::get<2>(a) << ") -> (" << std::get<0>(b) << ","
  //           << std::get<1>(b) << "," << std::get<2>(b) << ")" << std::endl;
  if (searchTimes == 10000) return false;
  searchTimes++;
  static const int dx[6] = {1, -1, 0, 0, 0, 0}, dy[6] = {0, 0, 1, -1, 0, 0},
                   dz[6] = {0, 0, 0, 0, 1, -1};

  auto move = [&](const T3 a, int k) {
    return T3{a[0] + dx[k], a[1] + dy[k], a[2] + dz[k]};
  };
  auto legal = [&](const T3 a) {
    if (a[2] < 1 || a[2] > space.chip.numLayer) return false;
    if (reached.find(a) != reached.end()) return false;
    return a[0] >= space.chip.gGridBoundaryIdx[0] &&
           a[0] <= space.chip.gGridBoundaryIdx[2] &&
           a[1] >= space.chip.gGridBoundaryIdx[1] &&
           a[1] <= space.chip.gGridBoundaryIdx[3];
  };
  assert(legal(a) && "Passed grid must be legal");

  int oppDir = lastDir ^ 1;
  passed.push_back(a);
  reached.insert(a);

  if (a == b) {
    std::cerr << "routing path found!" << std::endl;
    int i = 0, last = -1;
    for (auto x : passed) {
      int id{_addNode(x)};
      if (i > 0) _addEdge(last, id);
      i = i + 1;
      last = id;
    }
    return true;
  }

  std::vector<int> better, worse;
  auto prepareCandidate = [&](int k) {
    if (a[k] < b[k]) {
      if (k * 2 != oppDir) better.push_back(k * 2);
      if (k * 2 + 1 != oppDir) worse.push_back(k * 2 + 1);
    } else if (a[k] > b[k]) {
      if (k * 2 + 1 != oppDir) better.push_back(k * 2 + 1);
      if (k * 2 != oppDir) worse.push_back(k * 2);
    } else {
      if (k * 2 != oppDir) worse.push_back(k * 2);
      if (k * 2 + 1 != oppDir) worse.push_back(k * 2 + 1);
    }
  };

  if (basics.layer == "NoCstr" ||
      a[2] >= space.chip.layerName2Idx[basics.layer]) {
    if (space.chip.layers[a[2]].routingDir == db::H)
      prepareCandidate(1);
    else
      prepareCandidate(0);
  }

  if ((basics.layer != "NoCstr" &&
       a[2] < space.chip.layerName2Idx[basics.layer]) &&
      ((a[0] != b[0]) || (a[1] != b[1])))
    better.push_back(4);
  else if ((a[0] == b[0] && a[1] != b[1] && a[2] % 2 == 1) ||
           (a[1] == b[1] && a[0] != b[0] && a[2] % 2 == 0)) {
    better.push_back(4);
    if (basics.layer == "NoCstr" ||
        a[2] > space.chip.layerName2Idx[basics.layer])
      better.push_back(5);
  } else
    prepareCandidate(2);

  std::vector<std::pair<int, double>> toSort, order;

  if (better.size()) {
    for (auto x : better) {
      if (legal(move(a, x)))
        toSort.push_back(std::make_pair(x, _estCost(move(a, x), x)));
    }
    std::sort(toSort.begin(), toSort.end(),
              [&](const T2 &a, const T2 &b) { return a.second < b.second; });
    for (auto x : toSort)
      if (x.second < violationCost) order.push_back(x);
    toSort.clear();
  }

  if (worse.size()) {
    for (auto x : worse) {
      if (legal(move(a, x)))
        toSort.push_back(std::make_pair(x, _estCost(move(a, x), x)));
    }
    std::sort(toSort.begin(), toSort.end(),
              [&](const T2 &a, const T2 &b) { return a.second < b.second; });
    for (auto x : toSort)
      if (x.second < violationCost) order.push_back(x);
  }

  for (auto x : order) {
    if (_simpleRouteDFS(move(a, x.first), b, passed, reached, x.first))
      return true;
  }

  passed.pop_back();
  reached.erase(a);
  // std::cerr << "Route: (" << std::get<0>(a) << "," << std::get<1>(a) << ","
  //           << std::get<2>(a) << ") -> (" << std::get<0>(b) << ","
  //           << std::get<1>(b) << "," << std::get<2>(b) << ")"
  //           << "return false" << std::endl;
  return false;
}

/*
 * (1, 0, 0) : 0 , (-1, 0, 0) : 1
 * (0, 1, 0) : 2 , (0, -1, 0) : 3
 * (0, 0, 1) : 4 , (0, 0, -1) : 5
 */
void Net::_simpleRoute2Pins(const T3 a, const T3 b, const int lastDir) {
  std::cout << "a = (" << a[0] << ", " << a[1] << ", " << a[2] << ") b = ("
            << b[0] << ", " << b[1] << ", " << b[2] << ") lastDir = " << lastDir
            << std::endl;

  static const int dx[6] = {1, -1, 0, 0, 0, 0}, dy[6] = {0, 0, 1, -1, 0, 0},
                   dz[6] = {0, 0, 0, 0, 1, -1};

  auto move = [&](const T3 a, int k) {
    return T3{a[0] + dx[k], a[1] + dy[k], a[2] + dz[k]};
  };

  auto legal = [&](const T3 a) {
    if (a[2] < 1 || a[2] > space.chip.numLayer) return false;
    return a[0] >= space.chip.gGridBoundaryIdx[0] &&
           a[0] <= space.chip.gGridBoundaryIdx[2] &&
           a[1] >= space.chip.gGridBoundaryIdx[1] &&
           a[1] <= space.chip.gGridBoundaryIdx[3];
  };

  assert(legal(a) && "Passed grid must be legal");

  int oppDir = lastDir ^ 1;

  int ida{_addNode(a)};
  if (lastDir != -1) {
    int idLast{_getNodeIdx(move(a, oppDir))};
    _addEdge(idLast, ida);
  }

  if (a == b) {
    std::cerr << "arrived" << std::endl;
    return;
  }

  std::vector<int> better, worse;

  auto prepareCandidate = [&](int k) {
    if (a[k] < b[k]) {
      if (k * 2 != oppDir) better.push_back(k * 2);
      if (k * 2 + 1 != oppDir) worse.push_back(k * 2 + 1);
    } else if (a[k] > b[k]) {
      if (k * 2 + 1 != oppDir) better.push_back(k * 2 + 1);
      if (k * 2 != oppDir) worse.push_back(k * 2);
    } else {
      if (k * 2 != oppDir) worse.push_back(k * 2);
      if (k * 2 + 1 != oppDir) worse.push_back(k * 2 + 1);
    }
  };

  if (basics.layer == "NoCstr" ||
      a[2] >= space.chip.layerName2Idx[basics.layer]) {
    if (space.chip.layers[a[2]].routingDir == db::H)
      prepareCandidate(1);
    else
      prepareCandidate(0);
  }

  if ((basics.layer != "NoCstr" &&
       a[2] < space.chip.layerName2Idx[basics.layer]) &&
      ((a[0] != b[0]) || (a[1] != b[1])))
    better.push_back(4);
  else if ((a[0] == b[0] && a[1] != b[1] && a[2] % 2 == 1) ||
           (a[1] == b[1] && a[0] != b[0] && a[2] % 2 == 0)) {
    better.push_back(4);
    if (basics.layer == "NoCstr" ||
        a[2] > space.chip.layerName2Idx[basics.layer])
      better.push_back(5);
  } else
    prepareCandidate(2);

  std::vector<std::pair<int, double>> toSort;
  std::pair<int, double> bestInBetter, bestInWorse;

  if (better.size()) {
    std::cerr << "betters: ";
    for (auto x : better) std::cerr << x << " ";
    std::cerr << std::endl;
  }
  // First, try candidates in better
  for (auto x : better) {
    if (legal(move(a, x)))
      toSort.push_back(std::make_pair(x, _estCost(move(a, x), x)));
  }
  if (toSort.size()) {
    std::sort(toSort.begin(), toSort.end(),
              [&](const T2 &a, const T2 &b) { return a.second < b.second; });

    bestInBetter = toSort[0];
    if (bestInBetter.second < violationCost) {
      _simpleRoute2Pins(move(a, bestInBetter.first), b, bestInBetter.first);
      return;
    }
  }

  std::cerr << "betters failed" << std::endl;
  toSort.clear();
  for (auto x : worse) {
    if (legal(move(a, x)))
      toSort.push_back(std::make_pair(x, _estCost(move(a, x), x)));
  }

  if (toSort.size()) {
    std::sort(toSort.begin(), toSort.end(),
              [&](const T2 &a, const T2 &b) { return a.second < b.second; });
    bestInWorse = toSort[0];

    if (bestInWorse.second >= violationCost) {
      std::cerr << "Have to go back at (" << a[0] << ", " << a[1] << ", "
                << a[2] << ") " << oppDir << std::endl;
      _simpleRoute2Pins(move(a, oppDir), b, oppDir);
      return;
    } else {
      _simpleRoute2Pins(move(a, bestInWorse.first), b, bestInWorse.first);
      return;
    }
  }
}

using MST = std::vector<std::pair<int, int>>;

MST Prim(int n, int xs[], int ys[]) {
  MST ret;
  std::list<int> tree, waiting;
  tree.push_back(0);
  for (int i = 1; i < n; i++) waiting.push_back(i);

  auto Distance = [&](const int &a, const int &b) {
    return (xs[a] > xs[b] ? xs[a] - xs[b] : xs[b] - xs[a]) +
           (ys[a] > ys[b] ? ys[a] - ys[b] : ys[b] - ys[a]);
  };
  for (int i = 1; i < n; i++) {
    int min = std::numeric_limits<int>::max();
    std::pair<int, int> candi{-1, -1};
    auto toErase = waiting.begin();

    for (auto it = waiting.begin(); it != waiting.end(); it++) {
      int x = *it;
      for (auto y : tree) {
        int d = Distance(x, y);
        if (d < min) {
          min = d;
          candi = std::make_pair(y, x);
          toErase = it;
        }
      }
    }
    ret.push_back(candi);
    tree.push_back(candi.second);
    waiting.erase(toErase);
  }

  return ret;
}

bool Net::_simpleRoute(cf::Config &config, bool dfs) {
  std::cout << "simpleRoute for " << basics.netName << std::endl;
  std::vector<T3> pins(_getNetPins());
  std::map<T2, std::vector<int>> loc2RNode;
  std::map<int, T3> routeNodes;
  std::map<T2, int> loc2ID;
  std::set<int> newLocs;
  std::map<int, T2> id2Loc;
  std::vector<std::set<int>> locEdges;

  int numRNodes = 0;
  int numLocs = 0;

  // flute
  int degree = pins.size();
  int xs[100 * degree];
  int ys[100 * degree];
  int pt_cnt = 0;
  int node_cnt = 0;

  auto addNode = [&](int x, int y, int z) {
    routeNodes[numNodes] = {x, y, z};
    return numNodes++;
  };

  auto addLoc2Node = [&](int x, int y, int id) {
    auto ptr = loc2RNode.find({x, y});
    if (ptr == loc2RNode.end()) loc2RNode[{x, y}] = std::vector<int>();
    loc2RNode[{x, y}].push_back(id);
  };

  auto addLoc = [&](int x, int y) {
    auto ptr = loc2ID.find({x, y});
    if (ptr == loc2ID.end()) {
      loc2ID[{x, y}] = numLocs;
      id2Loc[numLocs++] = {x, y};
    }
    return loc2ID[{x, y}];
  };

  auto isNewLoc = [&](int x, int y) {
    auto ptr = loc2ID.find({x, y});
    return ptr == loc2ID.end();
  };

  auto append = [&](int x) {
    while (locEdges.size() <= x) locEdges.push_back(std::set<int>());
  };

  auto addEdge = [&](std::tuple<int, int> a, std::tuple<int, int> b) {
    int x1{std::get<0>(a)}, y1{std::get<1>(a)};
    int x2{std::get<0>(b)}, y2{std::get<1>(b)};
    int loc1{loc2ID[{x1, y1}]}, loc2{loc2ID[{x2, y2}]};

    if (loc1 != loc2) {
      append(loc1);
      append(loc2);
      locEdges[loc1].insert(loc2);
      locEdges[loc2].insert(loc1);
      // std::cerr << "Add edge: " << loc1 << " -> " << loc2 << std::endl;
      std::cerr << "Add edge: (" << x1 << ", " << y1 << ") -> (" << x2 << ", "
                << y2 << ")" << std::endl;
    }
  };

  auto changeRNodeZ = [&](int id, int z) {
    auto t3 = routeNodes[id];
    routeNodes[id] = T3{t3[0], t3[1], z};
  };

  std::sort(pins.begin(), pins.end());
  auto last = std::unique(pins.begin(), pins.end());
  pins.erase(last, pins.end());

  for (auto pin : pins) {
    xs[pt_cnt] = pin[0];
    ys[pt_cnt] = pin[1];
    pt_cnt += 1;
    int id{addNode(pin[0], pin[1], pin[2])};
    addLoc(pin[0], pin[1]);
    addLoc2Node(pin[0], pin[1], id);
  }

  degree = pt_cnt;
  if (degree < 2) {
    int min_layer = space.chip.layerName2Idx[basics.layer];
    if (pins[0][2] < min_layer) {
      for (int layer = pins[0][2] + 1; layer <= min_layer; layer++) {
        T3 node = {pins[0][0], pins[0][1], layer};
        int n_id = _getNodeIdx(node);
        int demand = space._getDemandOnGrid(node);
        int supply = space._getSupplyOnGrid(node);
        if (demand > supply) return false;
        _getEdgeIdx(_getNodeIdx(T3{pins[0][0], pins[0][1], layer - 1}), n_id);
      }
    } else {
      T3 node = {pins[0][0], pins[0][1], min_layer};
      _getNodeIdx(node);
    }
    return true;
  }

  MST mst = Prim(degree, xs, ys);

  for (auto pr : mst)
    addEdge({xs[pr.first], ys[pr.first]}, {xs[pr.second], ys[pr.second]});

  /*
  std::cerr << "start flute" << std::endl;
  Tree fluteTree = flute(degree, xs, ys, ACCURACY);
  std::cerr << "degree: " << degree << std::endl;
  for (int i = 0; i < degree * 2 - 2; i++) {
    Branch &branch1 = fluteTree.branch[i];
    Branch &branch2 = fluteTree.branch[branch1.n];

    std::cerr << "flute edge : " << "(" << branch1.x << ", " << branch1.y
              << ")->(" << branch2.x << ", " << branch2.y << ")" << std::endl;
    std::tuple<int, int> fluteEdge[2]{std::make_tuple(branch1.x, branch1.y),
                                      std::make_tuple(branch2.x, branch2.y)};

    for (int j = 0; j < 2; j++) {
      std::tuple<int, int> &nodeLoc = fluteEdge[j];
      int x{std::get<0>(nodeLoc)}, y{std::get<1>(nodeLoc)};
      bool isINode = isNewLoc(x, y);
      if (isINode) {
        int loc = addLoc(x, y);
        int id = addNode(x, y, -1);
        newLocs.insert(loc);
        addLoc2Node(x, y, id);
      }
    }
    addEdge(fluteEdge[0], fluteEdge[1]);
  }
  */

  if (newLocs.size()) {
    std::cerr << "NewLocs: " << std::endl;
    for (auto loc : newLocs) {
      std::cout << loc << " : " << std::get<0>(id2Loc[loc]) << ", "
                << std::get<1>(id2Loc[loc]) << std::endl;
    }
  }

  bool someChange = true;
  while (someChange) {
    someChange = false;
    for (auto i : newLocs) {
      auto t2 = id2Loc[i];
      if (true) {
        assert(loc2RNode.size() == 1 && "New loc must have one RNode");
        auto id = loc2RNode[t2][0];
        // auto rnode = routeNodes[id];
        if (true) {
          std::vector<int> layers;
          for (auto y : locEdges[i])
            if (y != i) {
              auto t2_ = id2Loc[y];
              for (auto nodeid : loc2RNode[t2_]) {
                auto rnode_ = routeNodes[nodeid];
                if (rnode_[2] != -1) layers.push_back(rnode_[2]);
              }
            }
          if (layers.size() == 0) continue;

          int layer = *std::max_element(layers.begin(), layers.end());
          changeRNodeZ(id, layer);
          someChange = true;
        }
      }
    }
  }

  for (auto entry : routeNodes) _addNode(entry.second);

  for (auto pin : pins) {
    int id{_getNodeIdx({pin[0], pin[1], pin[2]})};
    nodes[id].isPin = 1;
  }

  std::vector<int> highest;
  for (int i = 0; i < numLocs; i++) {
    auto t2 = id2Loc[i];
    std::vector<int> layers;
    for (auto id : loc2RNode[t2]) {
      auto rnode = routeNodes[id];
      layers.push_back(rnode[2]);
    }
    std::sort(layers.begin(), layers.end(), [](int a, int b) { return a > b; });

    for (int i = 0; i + 1 < layers.size(); i++) {
      if (dfs) {
        std::vector<T3> passed;
        std::set<T3> reached;
        if (!_simpleRouteDFS(T3{t2.first, t2.second, layers[i]},
                             T3{t2.first, t2.second, layers[i + 1]}, passed,
                             reached))
          std::cerr << "cannot route here" << std::endl;
        return false;
      } else {
        std::cerr << "SimpleRoute2Pins: (" << t2.first << ", " << t2.second
                  << ", " << layers[i] << ") -> (" << t2.first << ", "
                  << t2.second << ", " << layers[i + 1] << ")" << std::endl;
        _simpleRoute2Pins(T3{t2.first, t2.second, layers[i]},
                          T3{t2.first, t2.second, layers[i + 1]});
      }
    }
    highest.push_back(layers[0]);
  }

  for (int i = 0; i < numLocs; i++) {
    auto t2 = id2Loc[i];
    for (auto y : locEdges[i])
      if (y > i) {
        auto t2_ = id2Loc[y];
        if (dfs) {
          std::vector<T3> passed;
          std::set<T3> reached;
          if (!_simpleRouteDFS(T3{t2.first, t2.second, highest[i]},
                               T3{t2_.first, t2_.second, highest[y]}, passed,
                               reached))
            return false;
        } else {
          std::cerr << "SimpleRoute2Pins: (" << t2.first << ", " << t2.second
                    << ", " << highest[i] << ") -> (" << t2_.first << ", "
                    << t2_.second << ", " << highest[y] << ")" << std::endl;
          _simpleRoute2Pins(T3{t2.first, t2.second, highest[i]},
                            T3{t2_.first, t2_.second, highest[y]});
          std::cerr << "SimpleRoute2Pins succeed" << std::endl;
        }
      }
  }
  return true;
}

bool Net::_aStarRoute(cf::Config &config) {
  // Not implemented
  return true;
}

bool Net::route(cf::Config &config, bool dfs) {
  // std::vector<T3> pins(_getNetPins());
  if (config.getMode())
    return _aStarRoute(config);
  else
    return _simpleRoute(config, dfs);
}

bool Net::reroute(cf::Config &config, bool dfs) {
  if (basics.netName == "N130") {
    FILE *f = fopen("27273", "a");
    fprintf(f, "here clean N130\n");
    fclose(f);
  }
  getBoundingBox();
  searchTimes = 0;
  std::cerr << "reroute net " << basics.netName << std::endl;
  rerouted = true;
  space.unsavedNets.insert(basics.netName);
  if (basics.netName == "N130") {
    FILE *f = fopen("27273", "a");
    fprintf(f, "here reroute N130\n");
    fclose(f);
  }
  bool ret = route(config, dfs);
  if (ret) {
    routes.clear();
    for (auto &edge : edges)
      routes.push_back(db::Route(nodes[edge.second.id1], nodes[edge.second.id2],
                                 basics.netName));
    if (basics.netName == "N130") {
      FILE *f = fopen("27273", "a");
      fprintf(f, "N130 reroute succeed\n");
      fclose(f);
    }
  }
  return ret;
}

void Net::backup() {
  bk_routes = routes;
  modifyCells();
}

void Net::writeBack() { space.chip.routes[basics.netName] = routes; }

int Net::getLength(std::string obj) {
  int len = 0;
  if (obj == "route") {
    if (rerouted) {
      if (routes.size() != 0) len += routes.size() + 1;
    } else {
      for (auto &route : routes) {
        len += std::abs(route.sRowIdx - route.eRowIdx) +
               std::abs(route.sColIdx - route.eColIdx) +
               std::abs(route.sLayIdx - route.eLayIdx);
      }
      if (len != 0) len += 1;
    }
  } else if (obj == "bk_route") {
    if (rerouted) {
      if (routes.size() != 0) len += routes.size() + 1;
    } else {
      for (auto &route : bk_routes) {
        len += std::abs(route.sRowIdx - route.eRowIdx) +
               std::abs(route.sColIdx - route.eColIdx) +
               std::abs(route.sLayIdx - route.eLayIdx);
      }
      if (len != 0) len += 1;
    }
  }

  return len;
}

void Net::getBoundingBox() {
  auto &net = space.chip.nets[basics.netName];
  int min_x = 10000, min_y = 10000, max_x = 0, max_y = 0;
  for (auto &pin : net.pins) {
    auto cellName = pin[0];
    auto &cell = space.cellInss[cellName];

    min_x = std::min(min_x, cell.rowIdx);
    max_x = std::max(max_x, cell.rowIdx);

    min_y = std::min(min_y, cell.colIdx);
    max_y = std::max(max_y, cell.colIdx);
  }

  boundingBox = std::make_pair(T2{min_x, max_x}, T2{min_y, max_y});
}

void Net::recover() {
  modifyCells();
  routes = bk_routes;
  constructGraph();
  bk_routes.clear();
}

}  // namespace rt
