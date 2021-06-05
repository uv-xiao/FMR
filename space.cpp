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
    std::string mc = cell.mcName;
    auto mc_ptr = chip.masterCells.find(mc);
    
    for (auto blockage : _getBlockagesFromCell(cell)) {
      auto layerIdx = chip.layerName2Idx[blockage.layer];
      _addBlockage2Cell({cell.rowIdx, cell.colIdx, layerIdx}, blockage);
    }
  }

  // add net demands to cell demands
  // store nets, which pass a cell
  for (auto ent : nets) {
    for (auto cell : ent.second->getOccupiedCells())
      _addNet2Cell(cell.first, ent.first);
  }
}

std::vector<db::Blockage> Space::_getBlockagesFromCell(
                                  const db::CellIns &cell) {
  std::string mc = cell.mcName;
  auto mc_ptr = chip.masterCells.find(mc);
  return mc_ptr->second.blockages;
}

void Space::_addBlockage2Cell(const T3 &b, const db::Blockage &blockage) {
  auto ptr= demands.find(b);
  if (ptr == demands.end())
    demands.insert({b, blockage.demand});
  else
    ptr->second += blockage.demand;
}

void Space::_addNet2Cell(const T3 &b, const std::string &netName) {
  auto ptr = demands.find(b);
  if (ptr == demands.end())
    demands.insert({b, 1});
  else
    ptr->second += 1;

  auto ptr2 = passByNets.find(b);
  if (ptr2 == passByNets.end()) {
    passByNets.insert({b, stringset()});
    passByNets[b].insert(netName);
  }
  else
    ptr2->second.insert(netName);
}
} // namespace rt