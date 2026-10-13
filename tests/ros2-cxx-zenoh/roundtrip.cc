// Generated C++ service and action clients + servers talking over native
// Zenoh in one process (no ROS install, no dora daemon). Run by
// scripts/ros2-cxx-zenoh-roundtrip.sh with `humble` or `rep2016`.
#include "dora-node-api.h"
#include "ros2-bridge/impl.h"
#include "ros2-bridge/msg/example_interfaces.h"

#include <cstring>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

static rust::Vec<int32_t> fibonacci(int32_t order)
{
    rust::Vec<int32_t> sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    for (int32_t i = 1; i < order; i++)
    {
        sequence.push_back(sequence[i] + sequence[i - 1]);
    }
    return sequence;
}

static void check(bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

// Two goals in flight, finished as Succeeded and Aborted, each with one
// feedback message; then a goal canceled by the client. Feedback, results
// and status must reach the client under the right goal id.
static void actions(rust::Box<Ros2Node> &node, CombinedEvents &events, Ros2QosPolicies qos)
{
    auto server = example_interfaces::create_Fibonacci_action_server(*node, "/", "fibonacci", qos, events);
    auto client = example_interfaces::create_Fibonacci_action_client(*node, "/", "fibonacci", actionqos_default(), events);
    client->wait_for_action(node);

    example_interfaces::Fibonacci_Goal goal{};
    goal.order = 5;
    auto succeeded = client->send_goal(goal);
    goal.order = 7;
    auto aborted = client->send_goal(goal);
    client->request_result(succeeded);
    client->request_result(aborted);

    int feedback = 0;
    int results = 0;
    while (feedback < 2 || results < 2)
    {
        auto event = events.next();
        if (server->matches(event))
        {
            auto goal_event = server->downcast(std::move(event));
            auto id = goal_event->get_goal_id();
            int32_t order = goal_event->get_goal().order;
            example_interfaces::Fibonacci_Feedback progress{};
            progress.sequence = fibonacci(1);
            server->publish_feedback(id, std::move(progress));
            example_interfaces::Fibonacci_Result result{};
            result.sequence = fibonacci(order);
            server->send_result(id, order == 5 ? ActionStatusEnum::Succeeded : ActionStatusEnum::Aborted, std::move(result));
        }
        else if (client->matches_feedback(event))
        {
            auto progress = client->downcast_feedback(std::move(event));
            check(progress->matches_goal(succeeded) || progress->matches_goal(aborted), "feedback for an unknown goal");
            check(progress->get_feedback().sequence.size() == 2, "wrong feedback");
            feedback++;
        }
        else if (client->matches_result(event))
        {
            auto result = client->downcast_result(std::move(event));
            bool first = result->matches_goal(succeeded);
            check(first || result->matches_goal(aborted), "result for an unknown goal");
            check(result->get_status() == (first ? ActionStatusEnum::Succeeded : ActionStatusEnum::Aborted), "wrong result status");
            check(result->get_result().sequence.size() == (first ? 6u : 8u), "wrong result");
            results++;
        }
        else if (client->matches_status(event))
        {
            client->downcast_status(std::move(event));
        }
        else
        {
            throw std::runtime_error("unexpected event");
        }
    }

    goal.order = 3;
    auto canceled = client->send_goal(goal);
    bool executing = false;
    bool canceling = false;
    bool finished = false;
    std::optional<rust::Box<ActionGoalId>> server_id;
    while (!finished)
    {
        auto event = events.next();
        if (server->matches(event))
        {
            auto goal_event = server->downcast(std::move(event));
            if (goal_event->get_goal().order == 3 && !executing)
            {
                server_id.emplace(goal_event->get_goal_id());
                executing = true;
                client->cancel_goal(canceled);
            }
        }
        else if (client->matches_status(event))
        {
            auto status = client->downcast_status(std::move(event));
            if (executing && !canceling && status->matches_goal(canceled) && status->get_status() == ActionStatusEnum::Canceling)
            {
                canceling = true;
                server->send_result(*server_id, ActionStatusEnum::Canceled, example_interfaces::Fibonacci_Result{});
                client->request_result(canceled);
            }
        }
        else if (client->matches_result(event))
        {
            auto result = client->downcast_result(std::move(event));
            check(canceling && result->matches_goal(canceled), "unexpected result");
            check(result->get_status() == ActionStatusEnum::Canceled, "canceled goal not reported as canceled");
            finished = true;
        }
        else if (!client->matches_feedback(event))
        {
            throw std::runtime_error("unexpected event");
        }
    }
}

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

    try
    {
        actions(node, events, qos);
    }
    catch (const std::exception &e)
    {
        std::cerr << argv[1] << ": action round trip failed: " << e.what() << std::endl;
        return 1;
    }
    std::cout << argv[1] << ": action goals, feedback, results and cancel ok" << std::endl;

    if (call_across_namespaces(*context, "/robot1", "/robot1", "") != 0 ||
        call_across_namespaces(*context, "/robot2", "", "/robot2") != 0)
    {
        return 1;
    }
    return 0;
}
