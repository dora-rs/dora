#include "../build/dora-node-api.h"

#include <iostream>
#include <vector>

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;
    unsigned char counter = 0;

    auto dora_node = init_dora_node();

    // Demonstrate runtime introspection: a node can ask the daemon
    // what its own declared inputs/outputs look like, and what the
    // full dataflow descriptor is, without re-parsing the yaml.
    try
    {
        auto config = node_config_json(dora_node.send_output);
        std::cout << "Node config: " << std::string(config) << std::endl;

        auto descriptor = dataflow_descriptor_json(dora_node.send_output);
        std::cout << "Dataflow descriptor length: " << descriptor.length() << std::endl;
    }
    catch (const rust::Error &e)
    {
        std::cerr << "Introspection failed: " << e.what() << std::endl;
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
        else if (ty == DoraEventType::NodeFailed)
        {
            auto failed = event_as_node_failed(std::move(event));
            std::cerr << "Node failed: source=" << std::string(failed.source_node_id)
                      << " error=" << std::string(failed.error) << std::endl;
        }
        else if (ty == DoraEventType::Reload)
        {
            std::cout << "Reload event received" << std::endl;
        }
        else
        {
            std::cerr << "Unknown event type " << static_cast<int>(ty) << std::endl;
        }
    }

    std::cout << "GOODBYE FROM C++ node (using Rust API)" << std::endl;

    return 0;
}
