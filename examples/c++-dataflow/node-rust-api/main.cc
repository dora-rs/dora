#include "../build/dora-node-api.h"

#include <iostream>
#include <vector>

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;
    unsigned char counter = 0;

    auto dora_node = init_dora_node();

    // Demonstrate try_next_event (non-blocking poll). Before the main
    // receive loop starts pulling events, the event queue is typically
    // empty so the poll should return an `Empty` marker rather than
    // blocking. `Empty` is distinct from `Timeout` (no timeout was set)
    // and from `AllInputsClosed` (stream still open).
    auto poll = try_next_event(dora_node.events);
    if (event_type(poll) == DoraEventType::Empty) {
        std::cout << "No event ready yet (non-blocking poll)" << std::endl;
    }

    for (int i = 0; i < 20; i++)
    {

        auto event = dora_node.events->next();
        auto ty = event_type(event);

        if (ty == DoraEventType::AllInputsClosed)
        {
            break;
        }
        else if (ty == DoraEventType::Input)
        {
            auto input = event_as_input(std::move(event));

            counter += 1;

            std::cout << "Received input " << std::string(input.id) << " (counter: " << (unsigned int)counter << ")" << std::endl;

            std::vector<unsigned char> out_vec{counter};
            rust::Slice<const uint8_t> out_slice{out_vec.data(), out_vec.size()};
            auto result = send_output(dora_node.send_output, "counter", out_slice);
            auto error = std::string(result.error);
            if (!error.empty())
            {
                std::cerr << "Error: " << error << std::endl;
                return -1;
            }
        }
        else
        {
            std::cerr << "Unknown event type " << static_cast<int>(ty) << std::endl;
        }
    }

    std::cout << "GOODBYE FROM C++ node (using Rust API)" << std::endl;

    return 0;
}
