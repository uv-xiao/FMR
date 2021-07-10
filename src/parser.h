#pragma once

#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "types.h"

namespace db {

enum direct { H, V };

struct Layer {
  std::string layerName;
  int idx;
  direct routingDir;
  int defaultSupply;
  double powerFactor;
  Layer() {}
  Layer(std::string layerName, int idx, direct routingDir, int defaultSupply,
        double powerFactor)
      : layerName(layerName),
        idx(idx),
        routingDir(routingDir),
        defaultSupply(defaultSupply),
        powerFactor(powerFactor) {}
};

struct NonDefaultSupplyGGrid {
  int rowIdx, colIdx, layIdx, diffValue;
  NonDefaultSupplyGGrid() {}
  NonDefaultSupplyGGrid(int rowIdx, int colIdx, int layIdx, int diffValue)
      : rowIdx(rowIdx), colIdx(colIdx), layIdx(layIdx), diffValue(diffValue) {}
};

struct Pin {
  std::string pinName;
  std::string layer;
  Pin() {}
  Pin(std::string pinName, std::string layer)
      : pinName(pinName), layer(layer) {}
};

struct Blockage {
  std::string blkgName;
  std::string layer;
  int demand;
  Blockage() {}
  Blockage(std::string blkgName, std::string layer, int demand)
      : blkgName(blkgName), layer(layer), demand(demand) {}
};

struct MasterCell {
  std::string cellName;
  int pinCount;
  int blkgCount;
  std::map<std::string, Pin> pins;
  std::vector<Blockage> blockages;
  MasterCell() {}
  MasterCell(std::string cellName, int pinCount, int blkgCount)
      : cellName(cellName), pinCount(pinCount), blkgCount(blkgCount) {}
};

struct CellIns {
  std::string insName;
  std::string mcName;
  int rowIdx;
  int colIdx;
  bool movable;
  CellIns() {}
  CellIns(std::string insName, std::string mcName, int rowIdx, int colIdx,
          bool movable)
      : insName(insName),
        mcName(mcName),
        rowIdx(rowIdx),
        colIdx(colIdx),
        movable(movable) {}
  operator rt::T2() const { return rt::T2{rowIdx, colIdx}; }
};

struct Net {
  std::string netName;
  int numPin;
  std::string layer;
  double weight;
  std::vector<std::array<std::string, 2>> pins;
  Net() {}
  Net(std::string netName, int numPin, std::string layer, double weight)
      : netName(netName), numPin(numPin), layer(layer), weight(weight) {}
};

struct VoltageArea {
  std::string name;
  int gGridCount;
  std::set<rt::T2> gGridIdx;
  int insCount;
  std::vector<std::string> insName;
  VoltageArea() {}
  VoltageArea(std::string name) : name(name) {}
};

struct Route {
  std::string netName;
  int sRowIdx, sColIdx, sLayIdx, eRowIdx, eColIdx, eLayIdx;
  Route() {}
  Route(int sRowIdx, int sColIdx, int sLayIdx, int eRowIdx, int eColIdx,
        int eLayIdx, std::string netName)
      : sRowIdx(sRowIdx),
        sColIdx(sColIdx),
        sLayIdx(sLayIdx),
        eRowIdx(eRowIdx),
        eColIdx(eColIdx),
        eLayIdx(eLayIdx),
        netName(netName) {}
  Route(rt::T3 s, rt::T3 e, std::string netName): netName(netName) {
    sRowIdx = std::get<0>(s);
    sColIdx = std::get<1>(s);
    sLayIdx = std::get<2>(s);

    eRowIdx = std::get<0>(e);
    eColIdx = std::get<1>(e);
    eLayIdx = std::get<2>(e);
  }
};

struct Chip {
  // Basics
  int maxCellMove;
  int gGridBoundaryIdx[4];

  // Layers
  int numLayer;
  std::map<std::string, int> layerName2Idx;
  std::map<int, Layer> layers;

  // Non-default supply
  int numNonDefaultSupplyGGrid = 0;
  std::vector<NonDefaultSupplyGGrid> nonDefaultSupplyGrids;
  std::map<rt::T3, int> pos2SupplyDelta;

  // Master cells
  int numMasterCell;
  // std::vector<MasterCell> MasterCells;
  std::map<std::string, MasterCell> masterCells;

  // Cell instances
  int numCellIns;
  // std::vector<CellIns> cellIns;
  std::map<std::string, CellIns> cellInss;

  // Nets
  int numNet;
  // std::vector<Net> nets;
  std::map<std::string, Net> nets;

  // Voltage areas
  int numVoltageArea;
  std::vector<VoltageArea> voltageAreas;

  // Routes
  int numRoute;
  std::map<std::string, std::vector<Route>> routes;
};

void parse(Chip &chip, const char *dir);

}  // namespace db