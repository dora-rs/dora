// Generated C++ service client + server, and a topic publisher + subscriber,
// talking over native Zenoh in one process (no ROS install, no dora daemon).
// Run by scripts/ros2-cxx-zenoh-roundtrip.sh with `humble` or `rep2016`.
#include "dora-node-api.h"
#include "ros2-bridge/impl.h"
#include "ros2-bridge/msg/example_interfaces.h"
#include "ros2-bridge/msg/std_msgs.h"

#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

// A relative name is under the node's namespace, as on DDS: one call between a
// server and a client on a node in `node_namespace`, one of them named
// relative to it (empty namespace) and the other absolute.
int call_across_namespaces(Ros2Context &context, const char *node_namespace,
                           const char *server_namespace, const char *client_namespace)
{
    auto events = empty_combined_events();
    auto qos = qos_default();
    qos.reliable = true;
    auto node = context.new_node(node_namespace, "cxx_zenoh_namespaced");
    auto server = create_service_server_example_interfaces_AddTwoInts(*node, server_namespace, "add_two_ints", qos, events);
    auto client = create_client_example_interfaces_AddTwoInts(*node, client_namespace, "add_two_ints", qos, events);
    client->wait_for_service(node);
    example_interfaces::AddTwoInts_Request request{};
    request.a = 2;
    request.b = 3;
    client->send_request(request);
    auto request_event = events.next();
    if (!server->matches(request_event))
    {
        std::cerr << node_namespace << ": request did not reach the server" << std::endl;
        return 1;
    }
    auto received = server->downcast(std::move(request_event));
    example_interfaces::AddTwoInts_Response response{};
    response.sum = received->get_request().a + received->get_request().b;
    server->send_response(received->get_id(), response);
    auto response_event = events.next();
    if (!client->matches(response_event) || client->downcast(std::move(response_event)).sum != 5)
    {
        std::cerr << node_namespace << ": wrong or missing response" << std::endl;
        return 1;
    }
    std::cout << "relative and absolute names meet under " << node_namespace << std::endl;
    return 0;
}

// The same for a topic: one message from a publisher to a subscriber on a node
// in `node_namespace`, one of them named relative to it and the other absolute.
int publish_across_namespaces(Ros2Context &context, const char *node_namespace,
                              const char *publisher_namespace, const char *subscriber_namespace)
{
    auto events = empty_combined_events();
    auto qos = qos_default();
    qos.reliable = true;
    auto node = context.new_node(node_namespace, "cxx_zenoh_topic_namespaced");
    auto subscriber_topic = create_topic_std_msgs_String(*node, subscriber_namespace, "chatter", qos);
    auto subscription = create_subscription(*node, subscriber_topic, qos, events);
    auto publisher_topic = create_topic_std_msgs_String(*node, publisher_namespace, "chatter", qos);
    auto publisher = create_publisher(*node, publisher_topic, qos);
    std_msgs::String message{};
    message.data = "hello";
    publisher->publish(message);
    auto event = events.next();
    if (!subscription->matches(event) || subscription->downcast(std::move(event)).data != "hello")
    {
        std::cerr << node_namespace << ": wrong or missing message" << std::endl;
        return 1;
    }
    std::cout << "relative and absolute topic names meet under " << node_namespace << std::endl;
    return 0;
}

// Two robots in different namespaces that both use the relative `chatter`
// must not get each other's messages. The `/robot1` message is published
// first, so with cross-talk it would be the first one the `/robot2`
// subscriber gets.
int keep_namespaces_apart(Ros2Context &context)
{
    auto events = empty_combined_events();
    auto qos = qos_default();
    qos.reliable = true;
    auto robot1 = context.new_node("/robot1", "cxx_zenoh_topic_robot1");
    auto robot2 = context.new_node("/robot2", "cxx_zenoh_topic_robot2");
    auto subscriber_topic = create_topic_std_msgs_String(*robot2, "", "chatter", qos);
    auto subscription = create_subscription(*robot2, subscriber_topic, qos, events);
    auto robot1_topic = create_topic_std_msgs_String(*robot1, "", "chatter", qos);
    auto robot1_publisher = create_publisher(*robot1, robot1_topic, qos);
    auto robot2_topic = create_topic_std_msgs_String(*robot2, "", "chatter", qos);
    auto robot2_publisher = create_publisher(*robot2, robot2_topic, qos);
    std_msgs::String message{};
    message.data = "robot1";
    robot1_publisher->publish(message);
    message.data = "robot2";
    robot2_publisher->publish(message);
    auto event = events.next();
    if (!subscription->matches(event))
    {
        std::cerr << "unexpected event on /robot2" << std::endl;
        return 1;
    }
    auto received = subscription->downcast(std::move(event)).data;
    if (received != "robot2")
    {
        std::cerr << "/robot2 got the message from " << std::string(received) << std::endl;
        return 1;
    }
    std::cout << "relative topic names in /robot1 and /robot2 stay apart" << std::endl;
    return 0;
}

int run(int argc, char **argv);

int main(int argc, char **argv)
{
    // A service that is never found throws from `wait_for_service`.
    try
    {
        return run(argc, argv);
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}

int run(int argc, char **argv)
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

    // Many requests in flight at once. They must reach the server in the
    // order C++ sent them. The server answers them in reverse, and the client
    // must still get the responses in send order: `downcast` carries no
    // request id, so order is how a C++ client matches them up.
    const int requests = 50;
    for (int i = 0; i < requests; i++)
    {
        example_interfaces::AddTwoInts_Request request{};
        request.a = i;
        request.b = 100;
        client->send_request(request);
    }
    std::vector<uint64_t> ids;
    while ((int)ids.size() < requests)
    {
        auto event = events.next();
        if (!server->matches(event))
        {
            std::cerr << "unexpected event before all requests arrived" << std::endl;
            return 1;
        }
        auto request_event = server->downcast(std::move(event));
        if (request_event->get_request().a != (int64_t)ids.size())
        {
            std::cerr << "server got request " << request_event->get_request().a << ", expected " << ids.size() << std::endl;
            return 1;
        }
        ids.push_back(request_event->get_id());
    }
    for (int i = requests - 1; i >= 0; i--)
    {
        example_interfaces::AddTwoInts_Response response{};
        response.sum = i + 100;
        server->send_response(ids[i], response);
    }
    for (int answered = 0; answered < requests;)
    {
        auto event = events.next();
        if (!client->matches(event))
        {
            std::cerr << "unexpected event while waiting for responses" << std::endl;
            return 1;
        }
        auto response = client->downcast(std::move(event));
        if (response.sum != 100 + answered)
        {
            std::cerr << "response " << answered << " has sum " << response.sum << ", expected " << 100 + answered << std::endl;
            return 1;
        }
        answered++;
    }
    std::cout << argv[1] << ": " << requests << " requests answered in reverse, delivered in order" << std::endl;

    // The client keeps at most 64 calls in flight. With 64 unanswered
    // requests at the server, further requests must wait in the client's
    // queue and go out once replies free a slot, not fail.
    const int in_flight = 64;
    const int extra = 10;
    for (int i = 0; i < in_flight; i++)
    {
        example_interfaces::AddTwoInts_Request request{};
        request.a = i;
        request.b = 1000;
        client->send_request(request);
    }
    std::vector<uint64_t> held;
    while ((int)held.size() < in_flight)
    {
        auto event = events.next();
        if (!server->matches(event))
        {
            std::cerr << "unexpected event while filling the client" << std::endl;
            return 1;
        }
        held.push_back(server->downcast(std::move(event))->get_id());
    }
    for (int i = in_flight; i < in_flight + extra; i++)
    {
        example_interfaces::AddTwoInts_Request request{};
        request.a = i;
        request.b = 1000;
        client->send_request(request);
    }
    for (int i = 0; i < in_flight; i++)
    {
        example_interfaces::AddTwoInts_Response response{};
        response.sum = i + 1000;
        server->send_response(held[i], response);
    }
    for (int answered = 0; answered < in_flight + extra;)
    {
        auto event = events.next();
        if (server->matches(event))
        {
            auto request_event = server->downcast(std::move(event));
            example_interfaces::AddTwoInts_Response response{};
            response.sum = request_event->get_request().a + request_event->get_request().b;
            server->send_response(request_event->get_id(), response);
        }
        else if (client->matches(event))
        {
            auto response = client->downcast(std::move(event));
            if (response.sum != 1000 + answered)
            {
                std::cerr << "response " << answered << " has sum " << response.sum << std::endl;
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
    std::cout << argv[1] << ": " << in_flight + extra << " requests past the in-flight limit answered" << std::endl;

    if (call_across_namespaces(*context, "/robot1", "/robot1", "") != 0 ||
        call_across_namespaces(*context, "/robot2", "", "/robot2") != 0 ||
        publish_across_namespaces(*context, "/", "", "/") != 0 ||
        publish_across_namespaces(*context, "/robot1", "", "/robot1") != 0 ||
        publish_across_namespaces(*context, "/robot2", "/robot2", "") != 0 ||
        keep_namespaces_apart(*context) != 0)
    {
        return 1;
    }
    return 0;
}
