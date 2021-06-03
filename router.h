#pragma once

#include "types.h"

namespace rt {

class Router {
private:
  /* data */
public:
  Router(const db::Chip &chip);
  void run();
  void print(FILE *f);
  ~Router();
};


} // namespace rt