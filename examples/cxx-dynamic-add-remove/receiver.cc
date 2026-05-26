// Receiver node for the `dora node` lifecycle E2E fixture (issue
// #1703, C++ variant). Counts inputs and logs every 10th one. The
// dataflow.yml wires `value: sender/value` AND `tick:
// dora/timer/millis/500` so `open_inputs` is never empty even after
// sender is stopped (avoids the synthesized
// `Event::Stop(StopCause::AllInputsClosed)` that would close the
// stream — see apis/c/node/src/lib.rs:80 + apis/rust/node EventStream
// gate).

extern "C" {
#include "node/node_api.h"
}

#include <cstdint>
#include <iostream>

int main() {
    void *ctx = init_dora_context_from_env();
    if (!ctx) {
        std::cerr << "[cxx receiver] init_dora_context_from_env failed\n";
        return 1;
    }
    uint64_t count = 0;
    while (true) {
        void *event = dora_next_event(ctx);
        if (event == nullptr) {
            break;
        }
        DoraEventType ty = read_dora_event_type(event);
        if (ty == DoraEventType_Input) {
            count += 1;
            if (count % 10 == 0) {
                char *id_ptr;
                std::size_t id_len;
                read_dora_input_id(event, &id_ptr, &id_len);
                std::cerr << "[cxx receiver] " << count
                          << " messages (latest input: "
                          << std::string(id_ptr, id_len) << ")\n";
            }
        }
        free_dora_event(event);
    }
    std::cerr << "[cxx receiver] done (" << count << " messages)\n";
    free_dora_context(ctx);
    return 0;
}
