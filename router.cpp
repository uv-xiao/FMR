#include "router.h"

namespace rt {

Router::Router(db::Chip &chip) : chip(chip), space(chip) {}
Router::~Router() {}

void Router::run(cf::Config config) {
  
}

void Router::print(FILE *f) {}
} // namespace rt