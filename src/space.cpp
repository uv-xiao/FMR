#include "space.h"

#include "net.h"
#include "parser.h"
#include "types.h"

namespace rt {

Space::Space(db::Chip &chip) : chip(chip) {
  _prepareCells();
  _prepareNetsFromChip();
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
      if (cell.rowIdx == 8 && cell.colIdx == 8 && layerIdx == 3) {
        FILE *f = fopen("debug_info", "a");
        fprintf(f, "cell %s's blockage at (8,8,3), cost %d\n",
                cell.insName.c_str(), blockage.demand);
      }
    }
  }

  // collect fixed cells and cells constraint in voltage areas
  for (auto &entry : cellInss) {
    if (!entry.second.movable)
      fixedCells.insert(entry.first);
    else
      movableCells.push_back(entry.first);
  }
  for (auto &area : chip.voltageAreas)
    for (auto &cellName : area.insName)
      if (fixedCells.find(cellName) == fixedCells.end()) {
        Cell2voltageArea.insert({cellName, area});
      }
  std::cerr << "fixed cell number = " << fixedCells.size();
  for (auto &cell : fixedCells) std::cerr << " " << cell;
  std::cerr << std::endl;

  std::cerr << "movable cell number = " << movableCells.size();
  for (auto &cell : movableCells) std::cerr << " " << cell;
  std::cerr << std::endl;

  std::cerr << "movable cell in voltage area number = "
            << Cell2voltageArea.size();
  for (auto &cell : Cell2voltageArea) std::cerr << " " << cell.first;
  std::cerr << std::endl;
  /*
    // add net demands to cell demands
    // store nets, which pass a cell
    for (auto ent : nets) {
      for (auto cell : ent.second->getOccupiedCells())
        _addNet2Grid(cell.first, ent.first);
    }
  */
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
  // exit(-1);
}

std::vector<db::Blockage> Space::_getBlockagesFromCell(
    const db::CellIns &cell) {
  std::string mc = cell.mcName;
  auto mc_ptr = chip.masterCells.find(mc);
  return mc_ptr->second.blockages;
}

void Space::_addNet2Grid(const T3 &b, const std::string &netName) {
  if (b == T3{8, 8, 3}) {
    if (netName == "N664") {
      FILE *f = fopen("883", "a");
      fprintf(f, "N664 before operation demand = %d\n", _getDemandOnGrid(b));
      fclose(f);
    }
  }
  auto ptr2 = passByNets.find(b);
  if (ptr2 == passByNets.end()) {
    passByNets.insert({b, stringset()});
    passByNets[b].insert(netName);
    _addDemandOnGrid(b, 1);
  } else if (ptr2->second.find(netName) == ptr2->second.end()) {
    ptr2->second.insert(netName);
    _addDemandOnGrid(b, 1);
  }
  if (b == T3{8, 8, 3}) {
    if (netName == "N664") {
      FILE *f = fopen("883", "a");
      fprintf(f, "N664 after operation demand = %d\n", _getDemandOnGrid(b));
      fclose(f);
    }
  }
}

void Space::_removeNetFromGrid(const T3 &b, const std::string &netName) {
  auto ptr2 = passByNets.find(b);
  if (ptr2 != passByNets.end()) {
    ptr2->second.erase(netName);
    if (ptr2->second.empty()) {
      passByNets.erase(b);
    }
    _addDemandOnGrid(b, -1);
  }
}

void Space::_moveCell(std::string cellName, T2 to) {
  auto &cell = cellInss[cellName];
  const auto &blkgs = _getBlockagesFromCell(cell);
  for (auto &blkg : blkgs) {
    int layer = chip.layerName2Idx[blkg.layer];
    if (cell.rowIdx == 8 && cell.colIdx == 8 && layer == 3) {
      FILE *f = fopen("883", "a");
      fprintf(f, "before op, demand = %d\n", _getDemandOnGrid(T3{8, 8, 3}));
      fprintf(f, "remove cell %s's blockage at (8,8,3), cost %d\n",
              cell.insName.c_str(), blkg.demand);
      fclose(f);
    }
    if (to.first == 8 && to.second == 8 && layer == 3) {
      FILE *f = fopen("883", "a");
      fprintf(f, "before op, demand = %d\n", _getDemandOnGrid(T3{8, 8, 3}));
      fprintf(f, "add cell %s's blockage at (8,8,3), cost %d\n",
              cell.insName.c_str(), blkg.demand);
      fclose(f);
    }
    _addDemandOnGrid(T3{cell.rowIdx, cell.colIdx, layer}, -blkg.demand);
    _addDemandOnGrid(T3{std::get<0>(to), std::get<1>(to), layer}, blkg.demand);
    if (cell.rowIdx == 8 && cell.colIdx == 8 && layer == 3 ||
        to.first == 8 && to.second == 8 && layer == 3) {
      FILE *f = fopen("883", "a");
      fprintf(f, "after op, demand = %d\n", _getDemandOnGrid(T3{8, 8, 3}));
      fclose(f);
    }
  }
  cell.rowIdx = std::get<0>(to);
  cell.colIdx = std::get<1>(to);

  unsavedCells.insert(cellName);
  std::cout << "move cell " << cellName << " to (" << cell.rowIdx << ", "
            << cell.colIdx << ")" << std::endl;
}

void Space::_addDemandOnGrid(const T3 &b, int delta) {
  auto ptr = demands.find(b);
  if (b[0] == 2 && b[1] == 6 && b[2] == 2) {
    FILE *f = fopen("debug_info", "a");
    fprintf(f, "demand add %d\n", delta);
  }
  if (ptr == demands.end()) {
    demands.insert({b, delta});
    return;
  }
  ptr->second += delta;
  if (ptr->second == 0) demands.erase(b);
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
  for (auto &layer : chip.layers) {
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

void Space::writeBack() {
  for (auto cellName : unsavedCells) {
    std::cerr << "write cell " << cellName << "back" << std::endl;
    chip.cellInss[cellName] = cellInss[cellName];
  }
  for (auto netName : unsavedNets) {
    std::cerr << "write net " << netName << "back" << std::endl;
    nets[netName]->writeBack();
  }
  unsavedCells.clear();
  unsavedNets.clear();
}

}  // namespace rt