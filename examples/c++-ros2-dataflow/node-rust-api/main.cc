#include "../build/dora-node-api.h"
#include "../build/dora-ros2-bindings.h"

#include <iostream>
#include <vector>
#include <random>

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;

    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto qos = qos_default();
    qos.durability = Ros2Durability::Volatile;
    qos.liveliness = Ros2Liveliness::Automatic;
    qos.reliable = true;
    qos.max_blocking_time = 0.1;

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/ros2_demo", "turtle_teleop");
    auto vel_topic = node->create_topic_geometry_msgs_Twist("/turtle1", "cmd_vel", qos);
    auto vel_publisher = node->create_publisher(vel_topic, qos);
    auto pose_topic = node->create_topic_turtlesim_Pose("/turtle1", "pose", qos);
    auto pose_subscription = node->create_subscription(pose_topic, qos, merged_events);

    std::random_device dev;
    std::default_random_engine gen(dev());
    std::uniform_real_distribution<> dist(0., 1.);

    auto service_qos = qos_default();
    service_qos.reliable = true;
    service_qos.max_blocking_time = 0.1;
    service_qos.keep_last = 1;
    auto add_two_ints = node->create_client_example_interfaces_AddTwoInts("/", "add_two_ints", service_qos, merged_events);
    add_two_ints->wait_for_service(node);

    auto received_ticks = 0;
    auto responses_received = 0;

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
            else if (ty == DoraEventType::Input)
            {
                auto input = event_as_input(std::move(dora_event));
                received_ticks += 1;

                std::cout << "Received input " << std::string(input.id) << std::endl;

                geometry_msgs::Twist twist = {
                    .linear = {.x = dist(gen) + 1, .y = 0, .z = 0},
                    .angular = {.x = 0, .y = 0, .z = (dist(gen) - 0.5) * 5.0}};
                vel_publisher->publish(twist);

                example_interfaces::AddTwoInts_Request request = {.a = 4, .b = 5};
                add_two_ints->send_request(request);
            }
            else
            {
                std::cerr << "Unknown event type " << static_cast<int>(ty) << std::endl;
            }

            if (received_ticks > 20)
            {
                break;
            }
        }
        else if (pose_subscription->matches(event))
        {
            auto pose = pose_subscription->downcast(std::move(event));
            std::cout << "Received pose x:" << pose.x << ", y:" << pose.y << std::endl;
        }
        else if (add_two_ints->matches(event))
        {
            auto response = add_two_ints->downcast(std::move(event));
            assert(response.sum == 9);
            std::cout << "Received correct sum response from add_two_ints" << std::endl;
            responses_received += 1;
        }
        else
        {
            std::cout << "received unexpected event" << std::endl;
        }
    }

    std::cout << "Received " << responses_received << " service responses" << std::endl;
    assert(responses_received > 0);

    // try to access a constant for testing
    assert((sensor_msgs::const_NavSatStatus_STATUS_NO_FIX() == -1));

    std::cout << "GOODBYE FROM C++ node (using Rust API)" << std::endl;

    return 0;
}
