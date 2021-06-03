#include "parser.h"
#include "router.h"
#include "assert.h"

int main(int argc, char **argv) {
  assert(argc >= 2 && "At least 1 arguments as input, stdout as default output");
  db::Chip initialChip;
  db::parse(initialChip, argv[1]);

  rt::Router router{initialChip};
  router.run();

  if (argc == 2) router.print(stdout);
  else {
    FILE *f = fopen(argv[2], "w");
    router.print(f);
  }

  return 0;
}