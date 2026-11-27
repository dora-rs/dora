#include "dora-node-api.h"
#include "ros2-bridge/impl.h"

#include <cassert>
#include <iostream>

// Standalone C++ parameter verification node
int main()
{
    std::cout << "HELLO FROM C++ parameter node" << std::endl;

    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/demo_params", "param_node");

    // Local parameter hosting on Ros2Node
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

    // Update parameter
    node->set_parameter_double("speed", 2.0);
    assert(node->get_parameter_double("speed") == 2.0);
    std::cout << "C++ parameter hosting verified successfully" << std::endl;

    return 0;
}
