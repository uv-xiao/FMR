#pragma once

#include "types.h"
#include "parser.h"

#include <memory>
#include <cstring>
#include <string>
#include <vector>
#include <set>
#include <map>


namespace rt {

// Space is the 3D grid, where routing happens
class Space {
friend Net;
private:
  // store initial chip informations
  db::Chip &chip;
  
  // Store demand information
  std::map<T3, int> demands;

  // Store passing net for each grid
  std::map<T3, stringset> passByNets;
  
  // Store current cell instance places
  std::map<std::string, db::CellIns> cellInss;

  // Store Nets, which contains routing results
  std::map<std::string, net_sptr> nets;
  
  void _prepareNetsFromChip();
  
  void _prepareCells();

  std::vector<db::Blockage> _getBlockagesFromCell(const db::CellIns &cell);
  void _addBlockage2Cell(const T3 &b, const db::Blockage &blockage);
  void _addNet2Cell(const T3 &b, const std::string &netName);
  
public:
  Space() = default;
  // build a space with given chip
  Space(db::Chip &chip);

};

} // namespace rt
