#include "dora-node-api.h" // adjust this path if necessary

#include <iostream>
#include <vector>

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;
    unsigned char counter = 0;

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
            counter += 1;

            std::cout << "Received input " << std::string(input.id)
                      << " (counter: " << (unsigned int)counter << ")" << std::endl;

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
    }

    return 0;
}
