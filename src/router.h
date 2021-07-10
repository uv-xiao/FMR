#pragma once

#include "types.h"
#include "space.h"
#include "net.h"
#include "parser.h"
#include "config.h"

namespace rt {

class Router {
private:
  /* data */
  db::Chip &chip;
  Space space;

public:
  Router() = delete;
  Router(db::Chip &chip);
  void run(cf::Config);
  void print(FILE* f);
  ~Router();
};


} // namespace rt