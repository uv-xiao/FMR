#include "parser.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>

namespace db {

void splitline(std::string line, std::vector<std::string>& tokens) {
  tokens.clear();
  std::string token = "";
  for(auto c: line) {
    if (c == ' ') {
      tokens.push_back(token);
      token = "";
    } 
    else if (c == '/') {
      tokens.push_back(token);
      tokens.push_back("/");
      token = "";
    } 
    else token += c;
  }
  if (token.size() > 0)
    tokens.push_back(token);
}

void parse(Chip &chip, const char *dir = nullptr) {

  std::ifstream in(dir);
  std::string line;
  std::vector<std::string> tokens;

  std::string curMCName, curCIName, curNetName, curVAName;

  while(std::getline(in, line)) {
    splitline(line, tokens);

    if (tokens[0] == "MaxCellMove") {
      chip.maxCellMove = std::stoi(tokens[1]);
    } 
    else if (tokens[0] == "GGridBoundaryIdx") {
      for (int i = 0; i < 4; i++)
        chip.gGridBoundaryIdx[i] = std::stoi(tokens[i+1]);
    } 
    else if (tokens[0] == "NumLayer") {
      chip.numLayer = std::stoi(tokens[1]);
    } 
    else if (tokens[0] == "Lay") {
      std::string layerName = tokens[1];
      int idx = std::stoi(tokens[2]);
      direct routingDir = tokens[3] == "H" ? H : V;
      int defaultSupply = std::stoi(tokens[4]);
      double powerFactor = std::stof(tokens[5]);
      chip.layerName2Idx[layerName] = idx;
      chip.layers.insert({idx, Layer(layerName, idx, routingDir, 
                                     defaultSupply, powerFactor)});
    } 
    else if (tokens[0] == "NumNonDefaultSupplyGGrid") {
      chip.numNonDefaultSupplyGGrid = std::stoi(tokens[1]);
    } 
    else if (chip.nonDefaultSupplyGrids.size() < chip.numNonDefaultSupplyGGrid) {
      int rowIdx, colIdx, layIdx, diffValue;
      rowIdx = std::stoi(tokens[0]);
      colIdx = std::stoi(tokens[1]);
      layIdx = std::stoi(tokens[2]);
      diffValue = std::stoi(tokens[3]);
      chip.nonDefaultSupplyGrids.push_back(
          NonDefaultSupplyGGrid(rowIdx, colIdx, layIdx, diffValue)); 
      chip.pos2SupplyDelta.insert({rt::T3{rowIdx, colIdx, layIdx}, diffValue});           
    } 
    else if (tokens[0] == "NumMasterCell") {
      chip.numMasterCell = std::stoi(tokens[1]);
    } 
    else if (tokens[0] == "MasterCell") {
      std::string cellName = tokens[1];
      int pinCount = std::stoi(tokens[2]);
      int blkgCount = std::stoi(tokens[3]);
      chip.masterCells.insert({cellName, 
                               MasterCell(cellName, pinCount, blkgCount)} );
      curMCName = cellName;
    } 
    else if (tokens[0] == "Pin") {
      if (tokens[2] != "/") {
        // Here is pin for master cell
        Pin pin{tokens[1], tokens[2]};
        chip.masterCells[curMCName].pins.insert(
            std::make_pair(tokens[1], pin));
      } else {
        // Here, pins connected in net 
        chip.nets[curNetName].pins.push_back(
            std::array<std::string, 2>{tokens[1], tokens[3]});
      }
    } 
    else if (tokens[0] == "Blkg") {
      chip.masterCells[curMCName].blockages
          .push_back(Blockage(tokens[1], tokens[2], std::stoi(tokens[3])));
    } 
    else if (tokens[0] == "NumCellInst") {
      chip.numCellIns = std::stoi(tokens[1]);
    } 
    else if (tokens[0] == "CellInst") {
      int rowIdx = std::stoi(tokens[3]);
      int colIdx = std::stoi(tokens[4]);
      bool movable = tokens[5] == "Movable" ? true : false;
      curCIName = tokens[1];
      chip.cellInss.insert({tokens[1], CellIns(tokens[1], tokens[2], 
                                                rowIdx, colIdx, movable)});
    } 
    else if (tokens[0] == "NumNets") {
      chip.numNet = std::stoi(tokens[1]);
    } 
    else if (tokens[0] == "Net") {
      int numPin = std::stoi(tokens[2]);
      double weight = std::stof(tokens[4]);
      curNetName = tokens[1];
      chip.nets.insert({curNetName, Net(tokens[1], numPin, tokens[3], weight)});
    } 
    else if (tokens[0] == "NumRoutes") {
      chip.numRoute = std::stoi(tokens[1]);
    } 
    else if (tokens.size() == 7) {
      int sr, sc, sl, er, ec, el;
      sr = std::stoi(tokens[0]);
      sc = std::stoi(tokens[1]);
      sl = std::stoi(tokens[2]);
      er = std::stoi(tokens[3]);
      ec = std::stoi(tokens[4]);
      el = std::stoi(tokens[5]);
      if (chip.routes.count(tokens[6]) == 0)
        chip.routes.insert({tokens[6], std::vector<Route>()});
      chip.routes[tokens[6]].push_back(Route(sr, sc, sl, 
                                             er, ec, el, tokens[6]));
    } 
    else if (tokens[0] == "NumVoltageAreas") {
      chip.numVoltageArea = std::stoi(tokens[1]);
    } 
    else if (tokens[0] == "Name") {
      curVAName = tokens[1];
      chip.voltageAreas.push_back(VoltageArea(tokens[1]));
    } 
    else if (tokens[0] == "GGrids") {
      chip.voltageAreas.back().gGridCount = std::stoi(tokens[1]);
    } 
    else if (tokens[0] == "Instances") {
      chip.voltageAreas.back().insCount = std::stoi(tokens[1]);
    } 
    else if (tokens.size() == 2) {
      int rowIdx = std::stoi(tokens[0]);
      int colIdx = std::stoi(tokens[1]);
      chip.voltageAreas.back().gGridIdx.insert(
            rt::T2{rowIdx, colIdx});
    } 
    else if (tokens.size() == 1) {
      chip.voltageAreas.back().insName.push_back(tokens[0]);
    }
  }
}

} // namespace db

// #define DEBUG_PARSE
#ifdef DEBUG_PARSE

int main() {
    db::Chip chip;
    db::parse(chip, "case1.txt");
    
    std::cout << "layers" << std::endl;
    if (chip.numLayer != chip.layers.size())
    exit(-1);
    for (auto layer: chip.layers) {
        std::string dir = layer.second.routingDir == db::V ? "V" : "H";
        std::cout << layer.second.layerName << "," << layer.second.idx 
                  << "," << dir << "," << layer.second.defaultSupply 
                  << "," << layer.second.powerFactor << std::endl;
    }
    
    std::cout << "non default grids" << std::endl;
    if (chip.numNonDefaultSupplyGGrid != chip.nonDefaultSupplyGrids.size())
    exit(-1);
    for (auto grid: chip.nonDefaultSupplyGrids) {
        std::cout << grid.rowIdx << "," <<grid.colIdx << "," << grid.layIdx 
                  << "," <<grid.diffValue << std::endl;
    }
    
    std::cout << "master cells" <<std::endl;
    if (chip.numMasterCell != chip.masterCells.size())
      exit(-1);
    for (auto cell: chip.masterCells) {
        if (cell.second.pinCount != cell.second.pins.size())
        exit(-1);
        for (auto pin: cell.second.pins) {
            std::cout << pin.second.pinName << "," <<pin.second.layer << std::endl;
        }
        if (cell.second.blkgCount != cell.second.blockages.size())
        exit(-1);
        for (auto blkg: cell.second.blockages) {
            std::cout << blkg.blkgName << "," << blkg.layer << "," << blkg.demand << std::endl;
        }
    }

    std::cout << "cell instances" << std::endl;
    if (chip.numCellIns != chip.cellInss.size())
    exit(-1);
    for (auto ins: chip.cellInss) {
        std::string move = ins.second.movable == true ? "Movable" : "UnMovable";
        std::cout << ins.second.insName << "," <<ins.second.mcName 
                  << "," << ins.second.rowIdx << "," <<ins.second.colIdx 
                  << "," << move << std::endl;
    }

    std::cout << "nets" <<std::endl;
    if (chip.numNet != chip.nets.size())
    exit(-1);
    for (auto net: chip.nets) {
        std::cout << net.second.netName << "," << net.second.numPin
                  <<"," << net.second.layer <<"," << net.second.weight 
                  << std::endl;
        for (auto con: net.second.pins) {
            std::cout << con[0] <<"," <<con[1]<<std::endl;
        }
    }
    std::cout << "voltage area" << std::endl;
    if (chip.numVoltageArea != chip.voltageAreas.size())
    exit(-1);
    for (auto area: chip.voltageAreas) {
        if(area.gGridCount != area.gGridIdx.size())
        exit(-1);
        for(auto idx: area.gGridIdx) {
            std::cout << idx[0] <<"," <<idx[1] <<std::endl;
        }
        if(area.insCount != area.insName.size())
        exit(-1);
        for (auto ins: area.insName) {
            std::cout << ins << std::endl;
        }
    }
    
    return 0;
}

#endif