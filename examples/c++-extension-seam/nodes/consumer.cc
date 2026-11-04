// Consumer half of a minimal transport built on dora's extension seam.
//
// It receives a key, loads the descriptor stored under it, checks the bytes,
// and drops the entry, which notifies the producer.

#include <dora-node-api.h>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

// Both nodes must agree on these.
static const char *const NAMESPACE = "cxx-extension-example";
static const std::vector<std::uint8_t> DESCRIPTOR{1, 2, 3, 4};

int main()
{
    auto dora_node = init_dora_node();

    for (;;)
    {
        auto event = next_event(dora_node.events);
        const auto ty = event_type(event);

        if (ty == DoraEventType::Stop || ty == DoraEventType::AllInputsClosed)
        {
            std::cerr << "[consumer] stopped before a handle arrived" << std::endl;
            return -1;
        }
        if (ty != DoraEventType::Input)
        {
            continue;
        }

        std::string key;
        try
        {
            auto input = event_as_input(std::move(event));
            key.assign(input.data.begin(), input.data.end());
        }
        catch (const std::exception &e)
        {
            std::cerr << "[consumer] failed to read the handle: " << e.what() << std::endl;
            return -1;
        }

        auto loaded = extension_load(dora_node.send_output, NAMESPACE, key, false);
        if (!loaded.error.empty())
        {
            std::cerr << "[consumer] extension_load failed: " << std::string(loaded.error)
                      << std::endl;
            return -1;
        }
        if (!loaded.found)
        {
            std::cerr << "[consumer] no descriptor stored under " << key << std::endl;
            return -1;
        }
        if (!std::equal(loaded.value.begin(), loaded.value.end(), DESCRIPTOR.begin(),
                        DESCRIPTOR.end()))
        {
            std::cerr << "[consumer] descriptor for " << key << " has the wrong bytes"
                      << std::endl;
            return -1;
        }
        std::cout << "[consumer] loaded " << loaded.value.size() << " bytes for " << key
                  << std::endl;

        // Dropping notifies every node that stored or loaded the key.
        auto dropped = extension_drop(dora_node.send_output, NAMESPACE, key);
        if (!dropped.error.empty())
        {
            std::cerr << "[consumer] extension_drop failed: " << std::string(dropped.error)
                      << std::endl;
            return -1;
        }

        // A dropped key is absent, which is not an error.
        auto again = extension_load(dora_node.send_output, NAMESPACE, key, false);
        if (!again.error.empty())
        {
            std::cerr << "[consumer] loading " << key
                      << " after the drop failed: " << std::string(again.error) << std::endl;
            return -1;
        }
        if (again.found)
        {
            std::cerr << "[consumer] " << key << " is still loadable after the drop" << std::endl;
            return -1;
        }

        // Dropping an absent key succeeds, so a retry after a lost reply is safe.
        auto dropped_again = extension_drop(dora_node.send_output, NAMESPACE, key);
        if (!dropped_again.error.empty())
        {
            std::cerr << "[consumer] a second drop of " << key
                      << " failed: " << std::string(dropped_again.error) << std::endl;
            return -1;
        }

        std::cout << "[consumer] dropped " << key << std::endl;
        return 0;
    }
}
