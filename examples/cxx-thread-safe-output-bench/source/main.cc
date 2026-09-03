#include "../build/dora-node-api.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

// Benchmark source: emits `tick` (10 ms, payload = send timestamp) and
// `image` (1000 ms) from built-in timers.

// system_clock, not steady_clock: epoch is shared across processes.
static int64_t now_nanos()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

int main()
{
    auto dora_node = init_dora_node();

    while (true)
    {
        auto event = dora_node.events->next();
        auto ty = event_type(event);

        if (ty == DoraEventType::AllInputsClosed || ty == DoraEventType::Stop)
        {
            break;
        }
        else if (ty == DoraEventType::Input)
        {
            auto input = event_as_input(std::move(event));
            std::string id(input.id);

            if (id == "fast")
            {
                int64_t t = now_nanos();
                std::vector<uint8_t> payload(sizeof(t));
                std::memcpy(payload.data(), &t, sizeof(t));
                rust::Slice<const uint8_t> slice{payload.data(), payload.size()};
                send_output(dora_node.send_output, "tick", slice);
            }
            else if (id == "slow")
            {
                std::vector<uint8_t> payload{1};
                rust::Slice<const uint8_t> slice{payload.data(), payload.size()};
                send_output(dora_node.send_output, "image", slice);
            }
        }
    }

    return 0;
}
