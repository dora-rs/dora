#include "dora-node-api.h"
#include "ros2-bridge/impl.h"
#include "ros2-bridge/msg/example_interfaces.h"

#include <cassert>
#include <iostream>

// A dora C++ node hosting a ROS2 `AddTwoInts` service server. Mirrors the
// service-client usage in the turtle example, in reverse: instead of
// send_request + downcast(response), it matches incoming requests, reads the
// request, and replies via send_response(id, response). The `id` correlates the
// reply back to the ros2-client request on the Rust side.
int main()
{
    std::cout << "HELLO FROM C++ service server" << std::endl;

    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto service_qos = qos_default();
    service_qos.reliable = true;
    service_qos.max_blocking_time = 0.1;
    service_qos.keep_last = 1;

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/ros2_demo", "add_two_ints_server");
    auto add_server = create_service_server_example_interfaces_AddTwoInts(*node, "/", "add_two_ints", service_qos, merged_events);

    auto requests_served = 0;

    for (int i = 0; i < 1000; i++)
    {
        auto event = merged_events.next();

        if (event.is_dora())
        {
            auto dora_event = downcast_dora(std::move(event));
            auto ty = event_type(dora_event);

            if (ty == DoraEventType::AllInputsClosed)
            {
                break;
            }
            else if (ty == DoraEventType::Stop)
            {
                std::cout << "Received stop event" << std::endl;
                break;
            }
            // `tick` inputs just keep the node alive while it waits for requests.
        }
        else if (add_server->matches(event))
        {
            auto request_event = add_server->downcast(std::move(event));
            auto request = request_event->get_request();
            std::cout << "request: " << request.a << " + " << request.b << std::endl;

            example_interfaces::AddTwoInts_Response response = {.sum = request.a + request.b};
            add_server->send_response(request_event->get_id(), response);

            requests_served += 1;
            // The rclcpp minimal client is single-shot (sends one request and
            // exits), and ROS2 discovery is slow, so serve one request then exit.
            if (requests_served >= 1)
            {
                break;
            }
        }
        else
        {
            std::cout << "received unexpected event" << std::endl;
        }
    }

    std::cout << "Served " << requests_served << " service requests" << std::endl;
    assert(requests_served > 0);

    std::cout << "GOODBYE FROM C++ service server" << std::endl;

    return 0;
}
