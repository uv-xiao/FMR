#include "router.h"

#include <stdlib.h>
namespace rt {

Router::Router(db::Chip& chip) : chip(chip), space(chip) {}
Router::~Router() {}

void Router::run(cf::Config config) {
  srand(0);
  Move move(space, config);
  std::cerr << "error here" << std::endl;
  int failure = 0;
  while (true) {
    std::cerr << std::endl << std::endl;
    if (failure == 30) break;
    int method_selector = rand() % 10;
    if (method_selector < 7) {
      // P(big step) = 7/10
      int cellSelector;
      bool success;
      if (space.movedCells.size() < chip.maxCellMove) {
        cellSelector = rand() % space.movableCells.size();
        success = move.bigStep(space.movableCells[cellSelector]);
      } else {
        cellSelector = rand() % space.movedCells.size();
        int idx = 0;
        for (auto& cellName:space.movedCells) {
          if (idx == cellSelector) {
            success = move.bigStep(cellName);
            break;
          }
          idx++;
        }
      }
      if (!success) {
        failure++;
        std::cerr << "Fail " << failure << " times!" << std::endl;
      }
    } else if (method_selector < 9) {
      // P(small step) = 2/10
      int cellSelector;
      bool success;
      if (space.movedCells.size() < chip.maxCellMove) {
        cellSelector = rand() % space.movableCells.size();
        success = move.smallStep(space.movableCells[cellSelector]);
      } else {
        cellSelector = rand() % space.movedCells.size();
        int idx = 0;
        for (auto& cellName:space.movedCells) {
          if (idx == cellSelector) {
            success = move.smallStep(cellName);
            break;
          }
          idx++;
        }
      }
      if (!success) {
        failure++;
        std::cerr << "Fail " << failure << " times!" << std::endl;
      }
    }
    for (auto cell : space.movedCells) {
      std::cerr << "moved cell" << cell << std::endl;
    }
    // else {
    //   // P(net move) = 1/10
    //   int directionSelector = rand() % 2;
    //   move.netMove(directionSelector);
    // }
  }
  space.writeBack();
  for (auto& cell:space.cellInss) {
    T2 x = cell.second;
    if (x.first == 2 && x.second == 6) {
      std::cerr << "cell " << cell.second.insName << " at (2,6)" << std::endl;
    }
  }
  for (auto& net: space.nets) {
    if (net.second->_occupy(T3{2,6,2}))
      std::cerr << "net " << net.first << " cross (2,6,2)" << std::endl;
  }
}

void Router::print(FILE* f) {
  fprintf(f, "NumMovedCellInst %d\n", (int)space.movedCells.size());
  for (auto& cellName : space.movedCells) {
    auto& cell = space.chip.cellInss[cellName];
    fprintf(f, "CellInst %s %d %d\n", cellName.c_str(), cell.rowIdx,
            cell.colIdx);
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