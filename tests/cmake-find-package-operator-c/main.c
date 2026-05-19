// Smoke-test consumer for `find_package(dora-operator-api-c)`. The
// operator API is callback-shaped: the runtime loads our shared
// library and calls these three symbols. We supply no-op
// implementations so cmake configures, compiles, and links a
// shared library against `dora-operator-api-c::dora-operator-api-c`
// — failure to find the package or to resolve the type defs
// surfaces here before any real operator ever ships.

#include "operator_api.h"

#include <stddef.h>

EXPORT DoraInitResult_t dora_init_operator(void) {
    DoraInitResult_t result = {0};
    return result;
}

EXPORT DoraResult_t dora_drop_operator(void *operator_context) {
    (void)operator_context;
    DoraResult_t result = {0};
    return result;
}

EXPORT OnEventResult_t dora_on_event(
    RawEvent_t *event,
    const SendOutput_t *send_output,
    void *operator_context) {
    (void)event;
    (void)send_output;
    (void)operator_context;
    OnEventResult_t result = {0};
    return result;
}
