#pragma once

#include <memory>
#include <array>
#include <string>
#include <set>

namespace db {
struct Chip;
}

namespace rt {

class Net;


using chip_sptr = std::shared_ptr<db::Chip>;
using net_uptr = std::unique_ptr<Net>;
using T3 = std::array<int, 3>;
using T2 = std::pair<int, int>;
using stringset = std::set<std::string>;

} // namespace rt