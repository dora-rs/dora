// Smoke-test consumer for `find_package(dora-node-api-cxx)`. We do not
// actually call `init_dora_node` (that would require a live daemon) —
// taking the function's address proves the cxx bridge symbol resolves
// through the cmake config + xtask staging pipeline, which is what this
// test guards.

#include "dora-node-api.h"

int main() {
    auto init = &init_dora_node;
    return init == nullptr ? 1 : 0;
}
