#include "cxx.h"
#include "dora-node-api.h"
#include "ros2-bridge/impl.h"
#include "ros2-bridge/msg/example_interfaces.h"

#include <cassert>
#include <iostream>
#include <vector>

// A dora C++ node hosting a ROS2 `Fibonacci` action server. Mirrors the
// action-client example in reverse: instead of send_goal + downcast(result), it
// matches incoming goals, computes the Fibonacci sequence while publishing
// feedback, then sends the final result. A real rmw client cannot discover a
// ros2-client-hosted server (ros2-client#4), so the example pairs this server
// with a dora (ros2-client) action client in the same dataflow.
int main()
{
    std::cout << "HELLO FROM C++ action server" << std::endl;

    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto qos = qos_default();
    qos.durability = Ros2Durability::Volatile;
    qos.liveliness = Ros2Liveliness::Automatic;
    qos.reliable = true;
    qos.max_blocking_time = 0.1;
    qos.keep_last = 1;

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/ros2_demo", "fibonacci_action_server");
    auto server = example_interfaces::create_Fibonacci_action_server(
        *node, "/", "fibonacci", qos, merged_events);

    auto goals_served = 0;

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
            // `tick` inputs just keep the node alive while it waits for goals.
        }
        else if (server->matches(event))
        {
            auto goal_event = server->downcast(std::move(event));
            auto goal_id = goal_event->get_goal_id();
            auto order = goal_event->get_goal().order;
            std::cout << "received goal (order " << order << ")" << std::endl;

            // Fibonacci sequence, mirroring rclcpp's minimal_action_server: seed
            // [0, 1] then append `order` more terms, publishing the running
            // sequence as feedback.
            std::vector<std::int32_t> seq = {0, 1};
            for (std::int32_t k = 1; k < std::max(order, 1); k++)
            {
                seq.push_back(seq[k - 1] + seq[k]);
                example_interfaces::Fibonacci_Feedback feedback;
                for (auto v : seq)
                {
                    feedback.sequence.push_back(v);
                }
                server->publish_feedback(goal_id, feedback);
            }

            example_interfaces::Fibonacci_Result result;
            for (auto v : seq)
            {
                result.sequence.push_back(v);
            }
            server->send_result(goal_id, ::ActionStatusEnum::Succeeded, result);

            goals_served += 1;
            // The paired dora action client sends three goals; serve them then exit.
            if (goals_served >= 3)
            {
                break;
            }
        }
        else
        {
            std::cout << "received unexpected event" << std::endl;
        }
    }

    std::cout << "Served " << goals_served << " action goals" << std::endl;
    assert(goals_served > 0);

    std::cout << "GOODBYE FROM C++ action server" << std::endl;

    return 0;
}
