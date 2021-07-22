#include "config.h"

// #define DEBUG_CONFIG
#ifdef DEBUG_CONFIG
int main() {
  using namespace cf;
  Config config("./config.json");
  std::cout << config.getMode() << std::endl;
  std::cout << config.j["test"].get<int>() << std::endl;
  std::cout << config["test"] << std::endl;
  return 0;
}
#endif