// Filter node for the `dora node` lifecycle E2E fixture (issue
// #1703, C++ variant). Added dynamically via `dora node add
// --from-yaml filter-node.yml`. Counts inputs on `input` and emits a
// uint64 running total on `output`. Mirrors the Rust and Python
// filter shape so the cross-language matrix is consistent.

extern "C" {
#include "node/node_api.h"
}

#include <cstdint>
#include <cstring>
#include <iostream>

int main() {
    void *ctx = init_dora_context_from_env();
    if (!ctx) {
        std::cerr << "[cxx filter] init_dora_context_from_env failed\n";
        return 1;
    }
    uint64_t count = 0;
    const char *out_id = "output";

    while (true) {
        void *event = dora_next_event(ctx);
        if (event == nullptr) {
            break;
        }
        DoraEventType ty = read_dora_event_type(event);
        if (ty == DoraEventType_Input) {
            count += 1;
            int rc = dora_send_output(
                ctx, const_cast<char *>(out_id), std::strlen(out_id),
                reinterpret_cast<char *>(&count), sizeof(count));
            if (rc != 0) {
                std::cerr << "[cxx filter] dora_send_output failed\n";
                free_dora_event(event);
                break;
            }
        }
        free_dora_event(event);
    }
    free_dora_context(ctx);
    return 0;
}
