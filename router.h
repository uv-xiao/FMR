#pragma once

#include "types.h"
#include "space.h"
#include "net.h"
#include "parser.h"

namespace rt {

class Router {
private:
  /* data */
  db::Chip &chip;
  Space space;

public:
  Router(db::Chip &chip);
  void run();
  void print(FILE *f);
  ~Router();
};


} // namespace rt