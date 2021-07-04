#include "move.h"

#include "math.h"

const int INF = 0x3f3f3f3f;
namespace rt {

void Move::init() {
  for (auto& cell : space.chip.cellInss) {
    cell2nets.insert({cell.second.insName, rt::stringset()});
  }
  for (auto& net : space.chip.nets) {
    auto& netName = net.second.netName;
    net2cells.insert({netName, rt::stringset()});
    net2neighbors.insert({netName, rt::stringset()});
    for (auto& pin : net.second.pins) {
      auto& cellName = pin[0];
      cell2nets[cellName].insert(netName);
      net2cells[netName].insert(cellName);
    }
  }
  for (auto& netSet : cell2nets)
    for (const std::string& net1 : netSet.second)
      for (const std::string& net2 : netSet.second)
        if (net1 != net2) {
          auto iter = shareCells.find(stringpair(net1, net2));
          if (iter != shareCells.end()) {
            iter->second = 1;
            net2neighbors[net1].insert(net2);
          } else {
            iter->second += 1;
          }
        }
}

std::pair<T2, T2> Move::boundingBox(const std::string& netName,
                                    const std::string& exCellName) {
  auto net = space.chip.nets[netName];
  int min_x = INF, min_y = INF, max_x = 0, max_y = 0;
  for (auto& pin : net.pins) {
    auto cellName = pin[0];
    if (cellName == exCellName) continue;

    auto cell = space.chip.cellInss[cellName];

    min_x = std::min(min_x, cell.rowIdx);
    max_x = std::max(max_x, cell.rowIdx);

    min_y = std::min(min_y, cell.colIdx);
    max_y = std::max(max_y, cell.colIdx);
  }
  return std::make_pair(T2{min_x, max_x}, T2{min_y, max_y});
}

std::pair<T2, T2> Move::optimalRegion(const std::string& cellName) {
  std::vector<int> xs;
  std::vector<int> ys;

  for (auto& netName : cell2nets[cellName]) {
    auto box = boundingBox(netName, cellName);
    int lx = std::get<0>(box).first;
    int ux = std::get<1>(box).second;
    int ly = std::get<0>(box).first;
    int uy = std::get<1>(box).second;
    xs.push_back(lx);
    xs.push_back(ux);
    ys.push_back(ly);
    ys.push_back(uy);
  }
  std::sort(xs.begin(), xs.end());
  std::sort(ys.begin(), ys.end());
  std::pair<T2, T2> region =
      std::make_pair(T2{xs[xs.size() / 2 - 1], xs[xs.size() / 2]},
                     T2{ys[ys.size() / 2 - 1], ys[ys.size() / 2]});
  return region;
}

double Move::locCongest(T2 loc, double factor) {
  return space._sumLayer(loc, &Space::_getSupplyOnGrid) -
         factor * space._sumLayer(loc, &Space::_getDemandOnGrid);
}

double Move::computeCongest(std::pair<T2, T2> box, double factor) {
  int lx = std::get<0>(box).first, ux = std::get<0>(box).second;
  int ly = std::get<1>(box).first, uy = std::get<1>(box).second;
  double congest;
  for (int x = lx; x <= ux; x++)
    for (int y = ly; y <= uy; y++) {
      congest += locCongest(T2{x, y}, factor);
    }
  return congest;
}

T2 Move::computeBestCongestLoc(std::string cellName, std::pair<T2, T2> box,
                               double factor) {
  auto ptr = space.Cell2voltageArea.find(cellName);
  bool in_voltage_area = ptr != space.Cell2voltageArea.end();
  T2 Best_loc = space.cellInss[cellName];
  int lx = std::get<0>(box).first, ux = std::get<0>(box).second;
  int ly = std::get<1>(box).first, uy = std::get<1>(box).second;
  double bestCongest = 0;
  for (int x = lx; x <= ux; x++)
    for (int y = ly; y <= uy; y++) {
      T2 loc{x, y};
      if (in_voltage_area &&
          ptr->second.gGridIdx.find(loc) == ptr->second.gGridIdx.end())
        continue;
      double congest = locCongest(T2{x, y}, factor);
      if (congest > bestCongest) {
        bestCongest = congest;
        Best_loc = T2{x, y};
      }
    }
  return Best_loc;
}

void Move::bigStep() {
  std::string cellName;
  auto region = optimalRegion(cellName);
  T2 bestLoc = computeBestCongestLoc(cellName, region, 1.0);

  if (bestLoc != T2{space.cellInss[cellName]}) {
    space._moveCell(cellName, bestLoc);
    for (auto& netName : cell2nets[cellName]) {
      space.nets[netName]->reroute(conf);
    }
  }
}

void Move::netMove(int direction) {
  std::map<std::string, int> center;
  std::map<std::string, int> newCenter;
  std::map<std::string, double> congest;
  std::priority_queue<std::pair<double, std::string> >
      bestCongestNetQueue;  // (bestCongest, net)
  auto boxCenter = [&](std::pair<T2, T2> box, int direction) {
    int mid_x = (std::get<0>(box).first + std::get<0>(box).second) / 2;
    int mid_y = (std::get<1>(box).first + std::get<1>(box).second) / 2;
    return direction == 0 ? mid_x : mid_y;
  };

  for (auto& net : space.chip.nets) {
    auto& netName = net.second.netName;
    auto box = boundingBox(netName);
    center.insert({netName, boxCenter(box, direction)});
    congest.insert({netName, computeCongest(box)});
  }
  for (auto& net : space.chip.nets) {
    auto& netName = net.second.netName;
    newCenter[netName] = center[netName];
    if (congest[netName] < 0) {
      double bestCongest = congest[netName];
      for (auto net2 : net2neighbors[netName])
        bestCongest = std::max(bestCongest, congest[net2]);
      if (bestCongest < 0 || bestCongest == congest[netName]) continue;
      bestCongestNetQueue.push(std::make_pair(bestCongest, netName));
      for (auto net2 : net2neighbors[netName]) {
        newCenter[netName] += (center[net2] - center[netName]) *
                              (congest[net2] - congest[netName]) /
                              (bestCongest - congest[netName]) *
                              shareCells[stringpair(netName, net2)] /
                              net2cells.size();
      }
    }
  }

  auto computeBestNet2Move = [&](int max_n, double area_proportion) {
    stringset net2Move;
    auto& bound = space.chip.gGridBoundaryIdx;
    int area = 0, max_area = (bound[1] - bound[0]) * (bound[3] - bound[2]) *
                             area_proportion;
    while (net2Move.size() < max_n) {
      auto netName = bestCongestNetQueue.top().second;
      auto box = boundingBox(netName);
      area += (std::get<0>(box).second - std::get<0>(box).first) *
              (std::get<1>(box).second - std::get<1>(box).first);
      if (area > max_area) break;
      net2Move.insert(netName);
      bestCongestNetQueue.pop();
    }
    return net2Move;
  };

  auto computeCell2Move = [&](stringset& net2Move) {
    stringset cell2Move;
    for (auto& netName : net2Move) {
      for (auto& cell : net2cells[netName]) {
        if (space.fixedCells.find(cell) == space.fixedCells.end())
          cell2Move.insert(cell);
      }
    }
    return cell2Move;
  };

  auto net2Move = computeBestNet2Move(3, 0.3);
  auto cell2Move = computeCell2Move(net2Move);
  std::map<std::string, T2> moveLoc;
  for (auto& cellName : cell2Move) {
    auto& cell = space.cellInss[cellName];
    int coord_cell = (direction == 0) ? cell.rowIdx : cell.colIdx;
    std::vector<int> moveCandidate;
    for (auto& netName : cell2nets[cellName]) {
      if (net2Move.find(netName) != net2Move.end())
        moveCandidate.push_back(coord_cell + newCenter[netName] -
                                center[netName]);
    }
    int coord_max = 0, coord_min = INF;
    for (auto cand : moveCandidate) {
      coord_max = std::max(coord_max, cand);
      coord_min = std::min(coord_min, cand);
    }
    int middle = (coord_min + coord_max) / 2;
    int l = coord_min, r = coord_max;
    for (auto cand : moveCandidate) {
      if (cand > middle) r = std::min(r, cand);
      if (cand < middle) l = std::max(l, cand);
    }
    T2 bestLoc;
    if (direction == 0)
      bestLoc = computeBestCongestLoc(
          cellName, std::make_pair(T2{l, r}, T2{cell.colIdx, cell.colIdx}),
          1.0);
    else
      bestLoc = computeBestCongestLoc(
          cellName, std::make_pair(T2{cell.rowIdx, cell.rowIdx}, T2{l, r}),
          1.0);
    if (bestLoc != T2{cell}) moveLoc.insert({cellName, bestLoc});
  }
  for (auto pair : moveLoc) space._moveCell(pair.first, pair.second);

  stringset affectedNets;
  for (auto& pair : moveLoc)
    for (auto& netName : cell2nets[pair.first]) {
      affectedNets.insert(netName);
    }
  for (auto& netName : affectedNets) {
    space.nets[netName]->reroute(conf);
  }
}

}  // namespace rt