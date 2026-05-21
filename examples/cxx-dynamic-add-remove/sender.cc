// Sender node for the `dora node` lifecycle E2E fixture (issue #1703,
// C++ variant). Emits a monotonically-increasing uint64 counter on
// the `value` output every tick. Runs indefinitely until the harness
// tears down the dataflow (or until dora_next_event returns NULL,
// which the underlying EventStream signals after any Stop event).

extern "C" {
#include "node/node_api.h"
}

#include <cstdint>
#include <cstring>
#include <iostream>

int main() {
    void *ctx = init_dora_context_from_env();
    if (!ctx) {
        std::cerr << "[cxx sender] init_dora_context_from_env failed\n";
        return 1;
    }
    uint64_t counter = 0;
    const char *out_id = "value";

    while (true) {
        void *event = dora_next_event(ctx);
        if (event == nullptr) {
            // Stream closed (Stop received). Exit cleanly.
            break;
        }
        DoraEventType ty = read_dora_event_type(event);
        if (ty == DoraEventType_Input) {
            // Any input ID triggers a counter emit; the YAML wires
            // only `tick` as input so we don't need to discriminate.
            int rc = dora_send_output(
                ctx, const_cast<char *>(out_id), std::strlen(out_id),
                reinterpret_cast<char *>(&counter), sizeof(counter));
            if (rc != 0) {
                std::cerr << "[cxx sender] dora_send_output failed\n";
                free_dora_event(event);
                break;
            }
            counter += 1;
        }
        // Stop / InputClosed / Error / Unknown: just keep looping;
        // EventStream will return NULL on next call after Stop, which
        // ends the loop on the next iteration.
        free_dora_event(event);
    }
    free_dora_context(ctx);
    return 0;
}
