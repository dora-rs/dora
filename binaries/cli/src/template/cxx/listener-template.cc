#include "dora-node-api.h" // adjust this path if necessary

#include <iostream>
#include <vector>

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;
    unsigned char counter = 0;

    auto dora_node = init_dora_node();

    while (1)
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
            auto input_id = input.id;
            auto message = std::string(reinterpret_cast<const char*>(input.data.data()), input.data.size());
            std::cout << "I heard " << message << " from " << std::string(input_id) << std::endl;
        } 
        else {
            std::cerr << "Unknown event type " << static_cast<int>(ty) << std::endl;
        }
    }

    return 0;;
}
