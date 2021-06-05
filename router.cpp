#include "router.h"

namespace rt {

Router::Router(db::Chip &chip) : chip(chip), space(chip) {}

} // namespace rt