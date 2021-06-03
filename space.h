#pragma once

#include "types.h"
#include <memory>
#include <cstring>
#include <string>
#include <vector>
#include <set>
#include <map>


namespace rt {

using chip_sptr = std::shared_ptr<db::Chip>;
using net_sptr = std::unique_ptr<Net>;
using T3 = std::array<int, 3>;
using T2 = std::pair<int, int>;
using stringset = std::set<std::string>;

// Space is the 3D grid, where routing happens
class Space {
private:
  // store initial chip informations
  chip_sptr chip;
  
  // Store demand information
  std::map<T3, int> demands;

  // Store passing net for each grid
  std::map<T3, std::set<stringset>> passByNet;
  
  // Store current cell instance places
  std::map<std::string, T2> cellPlaces;

  // Store Nets, which contains routing results
  std::map<std::string, net_uptr> nets;
  

public:
  Space() = default;
  // build a space with given chip
  Space(const db::Chip &chip);

};

} // namespace rt
