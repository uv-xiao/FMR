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
  std::map<std::string, stringset> net2neighbors;
  std::map<std::string, stringset> cell2nets;
  std::map<std::string, stringset> net2cells;
  std::map<stringpair, int> shareCells;

  Move(Space& space, cf::Config& conf);

  // compute the bounding box of locations of cells inside the net,
  // and use exCellName to exclude the cell to move in "big step"
  // return ((lower row, upper row), lower column, upper column)
  std::pair<T2, T2> boundingBox(const std::string& netName,
                                const std::string& exCellName = "None");

  // compute the cell's optimal region to move using bounding
  // boxes of nets it connects to
  // return ((lower row, upper row), lower column, upper column)
  std::pair<T2, T2> optimalRegion(const std::string& cellName);

  // compute congest metric of a location (row, column pair)
  double locCongest(T2 loc, double factor = 1.0);

  // compute sum of congest metric in a region
  double computeCongest(std::pair<T2, T2> box, double factor = 1.0);

  // compute the location with largest congest metric in a region
  T2 computeBestCongestLoc(std::string cellName, std::pair<T2, T2> box,
                           double factor = 1.0);

  // move a cell to the best location of its optimal region,
  // and reroute impacted nets
  void bigStep(std::string cellName);

  // move a cell to the better location around its neighborhood
  // and reroute impacted nets
  void smallStep(std::string cellName);

  // move cells in several nets, and reroute all impacted nets
  void netMove(int direction);
};

}  // namespace rt