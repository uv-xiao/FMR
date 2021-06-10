#pragma once

#include "json.hpp"
#include <fstream>
#include <iostream>
#include <assert.h>

namespace cf {

using json = nlohmann::json;

class Config {
public:
  json j;
  Config() = delete;
  Config(const char * path = nullptr) {
    assert(path != nullptr && "config file path must be available");
    std::ifstream in(path);
    in >> j;
  }

  // 0: simple, 1: astar
  bool getMode() {
    assert(j.find("mode") != j.end() && "must specify routing mode");
    return j["mode"].get<std::string>() == "astar"; 
  }
  
  int getInt(const std::string &i) {
    return j[i].get<int>();
  }

  int operator[](const std::string i) {
    return getInt(i);
  }
  
};

}