#include "dora-node-api.h" // adjust this path if necessary

#include <iostream>
#include <vector>


int main()
{
    auto dora_node = init_dora_node();

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
            std::string message{"Hello World!"};
            rust::Slice<const uint8_t> message_slice{reinterpret_cast<const uint8_t*>(message.c_str()), message.size()};
            auto result = send_output(dora_node.send_output, "speech", message_slice);
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

    return 0;
}
