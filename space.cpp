#include "types.h"
#include "space.h"
#include "net.h"
#include "parser.h"

namespace rt {

Space::Space(db::Chip &chip) : chip(chip) {
  _prepareNetsFromChip();
  _prepareCells();
}


void Space::_prepareNetsFromChip() {
// Extract Nets from chip
  for (auto ent : chip.nets) {

    if (nets.find(ent.first) != nets.end())
      throw std::runtime_error("Nets with the same name, in Space constructer");
    
    // create the unique_ptr<Net>
    nets.insert({ent.first, std::make_shared<Net>(*this)});
    auto ptr = nets.find(ent.first);
    
    // set Net::basics
    ptr->second->setBasics(ent.second);
    
    // set Net::routes
    ptr->second->setRoutes(chip.routes[ent.first]);

    // construct graph(nodes, edges), and do simplification work
    ptr->second->constructGraph();
  }

}

void Space::_prepareCells() {
// Prepare cells from Net

  // store cell instances from chip
  cellInss = chip.cellInss;

  // add blockage demands to cell demands
  for (auto &entry : cellInss) {
    auto &cell = entry.second;
    for (auto blockage : _getBlockagesFromCell(cell)) {
      auto layerIdx = chip.layerName2Idx[blockage.layer];
      _addDemandOnGrid({cell.rowIdx, cell.colIdx, layerIdx}, blockage.demand);
    }
  }

  // add net demands to cell demands
  // store nets, which pass a cell
  for (auto ent : nets) {
    for (auto cell : ent.second->getOccupiedCells())
      _addNet2Grid(cell.first, ent.first);
  }
}

std::vector<db::Blockage> Space::_getBlockagesFromCell(
                                  const db::CellIns &cell) {
  std::string mc = cell.mcName;
  auto mc_ptr = chip.masterCells.find(mc);
  return mc_ptr->second.blockages;
}

void Space::_addNet2Grid(const T3 &b, const std::string &netName) {
  _addDemandOnGrid(b, 1);
  auto ptr2 = passByNets.find(b);
  if (ptr2 == passByNets.end()) {
    passByNets.insert({b, stringset()});
    passByNets[b].insert(netName);
  }
  else
    ptr2->second.insert(netName);
}

void Space::_removeNetFromGrid(const T3 &b, const std::string &netName) {
  _addDemandOnGrid(b, -1);
  auto ptr2 = passByNets.find(b);
  assert(ptr2 != passByNets.end() && "Net must be in passByNets");
  ptr2->second.erase(netName);
  if (ptr2->second.empty())
    passByNets.erase(b);
}
void Space::_moveCell(std::string cellName, T2 to) {
  auto& cell = cellInss[cellName];
  const auto& blkgs = _getBlockagesFromCell(cell);
  for (auto& blkg: blkgs) {
    int layer = chip.layerName2Idx[blkg.layer];
    _addDemandOnGrid(T3{cell.rowIdx, cell.colIdx, layer}, -blkg.demand);
    _addDemandOnGrid(T3{std::get<0>(to), std::get<1>(to), layer}, blkg.demand);
  }
  cell.rowIdx = std::get<0>(to);
  cell.colIdx = std::get<1>(to);
}

void Space::_addDemandOnGrid(const T3 &b, int delta = 1) {
  auto ptr = demands.find(b);
  if (ptr == demands.end()) {
    demands.insert({b, delta});
    return;
  }
  ptr->second += delta;
  if (ptr->second == 0)
    demands.erase(b);
  return;
}

int Space::_getDemandOnGrid(const T3 &b) {
  auto ptr = demands.find(b);
  if (ptr == demands.end()) {
    return 0;
  }
  return ptr->second;
}

int Space::_sumLayer(const T2 &b, int (Space::*getValue)(const T3 &b)) {
  int value = 0;
  for (auto& layer: chip.layers) {
    int layerIdx = layer.first;
    value += (this->*getValue)(T3{std::get<0>(b), std::get<1>(b), layerIdx});
  }
  return value;
}

int Space::_getSupplyOnGrid(const T3 &b) {
  auto ptr = chip.pos2SupplyDelta.find(b);
  int delta = (ptr != chip.pos2SupplyDelta.end()) ? ptr->second : 0;
  return chip.layers[std::get<2>(b)].defaultSupply + delta;
}

} // namespace rt