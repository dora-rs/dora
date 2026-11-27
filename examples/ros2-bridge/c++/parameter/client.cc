#include "dora-node-api.h"
#include "ros2-bridge/impl.h"

#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>

// Dora C++ node creating a parameter client to tune `/demo_params/param_host`.
int main()
{
    std::cout << "HELLO FROM C++ parameter client" << std::endl;

    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/tools", "param_client");

    std::cout << "[param_client] Waiting for parameter services on /demo_params/param_host..." << std::endl;
    auto client = node->create_parameter_client("/demo_params/param_host", 20.0);
    std::cout << "[param_client] Connected to target: " << client->target() << std::endl;

    // Read initial values
    double speed = client->get_double("speed", 5.0);
    std::cout << "[param_client] Read speed=" << speed << std::endl;
    assert(speed == 1.5);

    rust::String name = client->get_string("name", 5.0);
    std::cout << "[param_client] Read name=" << std::string(name) << std::endl;
    assert(std::string(name) == "robot");

    int64_t gain = client->get_int("gain", 5.0);
    std::cout << "[param_client] Read gain=" << gain << std::endl;
    assert(gain == 7);

    bool flag = client->get_bool("flag", 5.0);
    std::cout << "[param_client] Read flag=" << flag << std::endl;
    assert(flag == true);

    // List parameters
    auto param_names = client->list("", 0, 5.0);
    std::cout << "[param_client] Listed " << param_names.size() << " parameters" << std::endl;

    // Tune parameter
    std::cout << "[param_client] Tuning speed to 2.5..." << std::endl;
    bool set_ok = client->set_double("speed", 2.5, 5.0);
    assert(set_ok);

    // Verify updated value
    double new_speed = client->get_double("speed", 5.0);
    std::cout << "[param_client] Verified new speed=" << new_speed << std::endl;
    assert(new_speed == 2.5);

    // Check parameter type
    rust::String type_name = client->get_type("speed", 5.0);
    std::cout << "[param_client] Parameter speed type=" << std::string(type_name) << std::endl;

    // Check take_event
    auto event = client->take_event();
    if (event.has_event)
    {
        std::cout << "[param_client] Captured parameter event" << std::endl;
    }

    std::cout << "[param_client] All parameter client tests passed successfully" << std::endl;
    return 0;
}
