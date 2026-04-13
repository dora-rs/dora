#include "../build/dora-node-api.h"

#include <iostream>
#include <vector>

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;
    unsigned char counter = 0;

    auto dora_node = init_dora_node();

    auto id = node_id(dora_node.send_output);
    auto df_id = dataflow_id(dora_node.send_output);
    if (id.empty() || df_id.empty()) {
        std::cerr << "node_id() or dataflow_id() returned empty string" << std::endl;
        return -1;
    }
    if (std::string(id) != "cxx-node-rust-api") {
        std::cerr << "node_id() mismatch: expected 'cxx-node-rust-api', got '" << std::string(id) << "'" << std::endl;
        return -1;
    }

    // Demonstrate try_next_event (non-blocking poll)
    auto poll = try_next_event(dora_node.events);
    if (event_type(poll) == DoraEventType::Timeout) {
        std::cout << "No event ready yet (non-blocking)" << std::endl;
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
