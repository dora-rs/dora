// Producer half of a minimal transport built on dora's extension seam.
//
// It stores a descriptor in the daemon's extension table, sends only the key
// as a regular output, and then waits until it is told the key was dropped.
// See `docs/extensions.md` for what the seam guarantees.

#include <dora-node-api.h>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

// Both nodes must agree on these.
static const char *const NAMESPACE = "cxx-extension-example";
static const char *const KEY = "frame-1";
static const std::vector<std::uint8_t> DESCRIPTOR{1, 2, 3, 4};

// Fail instead of waiting forever if the drop notification never arrives.
static const int MAX_TICKS = 100;

int main()
{
    auto dora_node = init_dora_node();

    bool sent = false;
    int ticks = 0;

    for (;;)
    {
        auto event = next_event(dora_node.events);
        const auto ty = event_type(event);

        if (ty == DoraEventType::Stop || ty == DoraEventType::AllInputsClosed)
        {
            std::cerr << "[producer] stopped before the drop notification arrived" << std::endl;
            return -1;
        }
        if (ty != DoraEventType::Input)
        {
            continue;
        }

        if (!sent)
        {
            rust::Slice<const std::uint8_t> descriptor{DESCRIPTOR.data(), DESCRIPTOR.size()};
            auto stored = extension_store(dora_node.send_output, NAMESPACE, KEY, descriptor);
            if (!stored.error.empty())
            {
                std::cerr << "[producer] extension_store failed: " << std::string(stored.error)
                          << std::endl;
                return -1;
            }

            // The descriptor stays in the daemon; the output carries only its key.
            const std::string key(KEY);
            rust::Slice<const std::uint8_t> key_slice{
                reinterpret_cast<const std::uint8_t *>(key.data()), key.size()};
            auto result = send_output(dora_node.send_output, "handle", key_slice);
            if (!result.error.empty())
            {
                std::cerr << "[producer] failed to send the handle: "
                          << std::string(result.error) << std::endl;
                return -1;
            }
            std::cout << "[producer] stored " << KEY << " and sent its key" << std::endl;
            sent = true;
            continue;
        }

        // Drop notifications are not events: poll for them wherever the
        // transport next runs.
        for (const auto &dropped : drain_dropped_extension_keys(NAMESPACE))
        {
            if (std::string(dropped) == KEY)
            {
                std::cout << "[producer] " << KEY << " was dropped, releasing it" << std::endl;
                return 0;
            }
        }

        if (++ticks > MAX_TICKS)
        {
            std::cerr << "[producer] no drop notification for " << KEY << " after "
                      << MAX_TICKS << " ticks" << std::endl;
            return -1;
        }
    }
}
