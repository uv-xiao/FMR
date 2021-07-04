#include "router.h"

#include <stdlib.h>
namespace rt {

Router::Router(db::Chip &chip) : chip(chip), space(chip) {}
Router::~Router() {}

void Router::run(cf::Config config) {
  space._prepareCells();
  space._prepareNetsFromChip();
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
      move.bigStep(space.movableCells[cellSelector]);
    } else {
      // P(net move) = 1/10
      int directionSelector = rand() % 2;
      move.netMove(directionSelector);
    }
  }
}

void Router::print(FILE *f) {

}

}  // namespace rt