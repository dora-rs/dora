// Smoke-test consumer for `find_package(dora-node-api-c)`. We do not
// actually call `init_dora_context_from_env` (that would require a live
// daemon) — taking the function's address proves the symbol resolves
// through the cmake config + xtask staging pipeline, which is what this
// test guards.

#include "node_api.h"

#include <stddef.h>

int main(void) {
    void *(*init)(void) = init_dora_context_from_env;
    return init == NULL ? 1 : 0;
}
