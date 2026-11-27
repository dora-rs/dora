#include "dora-node-api.h"
#include "ros2-bridge/impl.h"

#include <cassert>
#include <iostream>

// Dora C++ node hosting parameters on `/demo_params/param_host`.
// The rcl_interfaces parameter services are hosted natively by ros2-client;
// this node initializes parameters and answers events.
int main()
{
    std::cout << "HELLO FROM C++ parameter host" << std::endl;

    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/demo_params", "param_host");

    // Local parameter hosting
    node->set_parameter_double("speed", 1.5);
    node->set_parameter_string("name", "robot");
    node->set_parameter_int("gain", 7);
    node->set_parameter_bool("flag", true);

    assert(node->has_parameter("speed"));
    assert(node->has_parameter("name"));
    assert(node->has_parameter("gain"));
    assert(node->has_parameter("flag"));
    assert(!node->has_parameter("missing"));

    assert(node->get_parameter_double("speed") == 1.5);
    assert(std::string(node->get_parameter_string("name")) == "robot");
    assert(node->get_parameter_int("gain") == 7);
    assert(node->get_parameter_bool("flag") == true);

    std::cout << "[param_host] Initial parameters declared and verified locally" << std::endl;

    for (int i = 0; i < 50; i++)
    {
        auto event = merged_events.next();
        if (event.is_dora())
        {
            auto dora_event = downcast_dora(std::move(event));
            auto ty = event_type(dora_event);
            if (ty == DoraEventType::AllInputsClosed || ty == DoraEventType::Stop)
            {
                std::cout << "[param_host] Received stop event" << std::endl;
                break;
            }
            if (ty == DoraEventType::Input)
            {
                // Print speed occasionally to observe remote updates from client
                double current_speed = node->get_parameter_double("speed");
                if (current_speed > 1.9)
                {
                    std::cout << "[param_host] Observed tuned parameter speed=" << current_speed << std::endl;
                    break;
                }
            }
        }
    }

    std::cout << "[param_host] Done" << std::endl;
    return 0;
}
