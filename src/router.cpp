#include "router.h"

#include <stdlib.h>
namespace rt {

Router::Router(db::Chip& chip) : chip(chip), space(chip) {}
Router::~Router() {}

void Router::run(cf::Config config) {
  Move move(space, config);
  while (space.movedCells.size() < chip.maxCellMove) {
    int method_selector = rand() % 10;
    if (method_selector < 7) {
      // P(big step) = 7/10
      int cellSelector = rand() % space.movableCells.size();
      move.bigStep(space.movableCells[cellSelector]);
    } else if (method_selector < 9) {
      // P(small step) = 2/10
      int cellSelector = rand() % space.movableCells.size();
      move.smallStep(space.movableCells[cellSelector]);
    } else {
      // P(net move) = 1/10
      int directionSelector = rand() % 2;
      move.netMove(directionSelector);
    }
  }
  space.writeBack();
}

void Router::print(FILE* f) {
  fprintf(f, "NumMovedCellInst %d\n", space.movedCells.size());
  for (auto& cell : chip.cellInss) {
    fprintf(f, "CellInst %s %d %d\n", cell.first.c_str(), cell.second.rowIdx,
            cell.second.colIdx);
  }

  int num_route = 0;
  for (auto& route_vec : chip.routes) {
    num_route += route_vec.second.size();
  }
  fprintf(f, "NumRoutes %d\n", num_route);
  for (auto& route_vec : chip.routes) {
    for (auto& route : route_vec.second) {
      fprintf(f, "%d %d %d %d %d %d %s\n", route.sRowIdx, route.sColIdx,
              route.sLayIdx, route.eRowIdx, route.eColIdx, route.eLayIdx,
              route.netName.c_str());
    }
  }
  fclose(f);
}

}  // namespace rt