#pragma once

#include <queue>

#include "config.h"
#include "net.h"
#include "parser.h"
#include "space.h"
#include "types.h"

namespace rt {

class Move {
 public:
  Space& space;
  cf::Config& conf;
  Move(Space& space, cf::Config& conf) : space(space), conf(conf){};
  void bigStep();
  void smallStep();
  std::pair<T2, T2> boundingBox(const std::string& netName,
                                const std::string& exCellName = "None");
  std::pair<T2, T2> optimalRegion(const std::string& netName);
  std::map<std::string, stringset> net2neighbors;
  std::map<std::string, stringset> cell2nets;
  std::map<std::string, stringset> net2cells;
  void init();
  void netMove(int direction);
  double locCongest(T2 loc, double factor = 1.0);
  std::map<stringpair, int> shareCells;
  double computeCongest(std::pair<T2, T2> box, double factor = 1.0);
};

class CellPoll {
 public:
  stringset moved;
  stringset unmoved;
  void selectCell2Move();
};

}  // namespace rt