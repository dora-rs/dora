#include "dora-node-api.h" // adjust this path if necessary

#include <iostream>
#include <vector>
#include <cstdint>

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;
    unsigned char counter = 0;

    auto dora_node = init_dora_node();

    while (1)
    {
        auto event = next_event(dora_node.events);
        if (event_type(event) != DoraEventType::Input)
        {
            break;
        }

        auto input = event_as_input(std::move(event));
        counter += 1;

        std::cout << "Received input " << std::string(input.id) << " (counter: " << (unsigned int)counter << ")" << std::endl;

        std::vector<uint8_t> out_vec{counter};
        auto result = send_output(dora_node.send_output, "counter", rust::Slice<const uint8_t>{out_vec.data(), out_vec.size()});
        auto error = std::string(result.error);
        if (!error.empty())
        {
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }
    }

    return 0;
}
