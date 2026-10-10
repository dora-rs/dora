// Generated C++ service client + server talking over native Zenoh in one
// process (no ROS install, no dora daemon). Run by
// scripts/ros2-cxx-zenoh-roundtrip.sh with `humble` or `rep2016`.
#include "dora-node-api.h"
#include "ros2-bridge/impl.h"
#include "ros2-bridge/msg/example_interfaces.h"

#include <cstring>
#include <iostream>

int main(int argc, char **argv)
{
    if (argc != 2 || (std::strcmp(argv[1], "humble") != 0 && std::strcmp(argv[1], "rep2016") != 0))
    {
        std::cerr << "usage: roundtrip humble|rep2016" << std::endl;
        return 2;
    }
    auto kind = std::strcmp(argv[1], "humble") == 0 ? Ros2TransportKind::ZenohHumble : Ros2TransportKind::ZenohRep2016;

    auto events = empty_combined_events();
    auto qos = qos_default();
    qos.reliable = true;
    qos.keep_last = 10;

    auto context = init_ros2_context_with_transport(Ros2TransportConfig{kind, ""});
    auto node = context->new_node("/", "cxx_zenoh_roundtrip");
    auto server = create_service_server_example_interfaces_AddTwoInts(*node, "/", "add_two_ints", qos, events);
    auto client = create_client_example_interfaces_AddTwoInts(*node, "/", "add_two_ints", qos, events);
    client->wait_for_service(node);

    // Several requests in flight at once, answered out of the merged event stream.
    const int requests = 5;
    for (int i = 0; i < requests; i++)
    {
        example_interfaces::AddTwoInts_Request request{};
        request.a = i;
        request.b = 100;
        client->send_request(request);
    }

    int answered = 0;
    while (answered < requests)
    {
        auto event = events.next();
        if (server->matches(event))
        {
            auto request_event = server->downcast(std::move(event));
            const auto &request = request_event->get_request();
            example_interfaces::AddTwoInts_Response response{};
            response.sum = request.a + request.b;
            server->send_response(request_event->get_id(), response);
        }
        else if (client->matches(event))
        {
            auto response = client->downcast(std::move(event));
            if (response.sum < 100 || response.sum >= 100 + requests)
            {
                std::cerr << "unexpected response " << response.sum << std::endl;
                return 1;
            }
            answered++;
        }
        else
        {
            std::cerr << "unexpected event" << std::endl;
            return 1;
        }
    }
    std::cout << argv[1] << ": " << answered << "/" << requests << " requests answered" << std::endl;
    return 0;
}
