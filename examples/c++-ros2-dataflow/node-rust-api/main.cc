#include "../build/dora-node-api.h"
#include "../build/dora-ros2-bindings.h"

#include <iostream>
#include <vector>
#include <random>

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;

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
    auto pose_subscription = node->create_subscription(pose_topic, qos);

    std::random_device dev;
    std::default_random_engine gen(dev());
    std::uniform_real_distribution<> dist(0., 1.);

    auto dora_node = init_dora_node();

    std::cout << "MERGING EVENTS" << std::endl;
    auto merged_events = merge_events(std::move(dora_node.events), event_stream(std::move(pose_subscription)));
    std::cout << "MERGED EVENTS" << std::endl;

    auto received_ticks = 0;

    for (int i = 0; i < 1000; i++)
    {
        auto event = merged_events->next();

        if (is_ros2(event))
        {
            auto ros2_event = downcast_ros2(std::move(event));
            if (turtlesim::is_Pose(ros2_event))
            {
                auto pose = turtlesim::downcast_Pose(std::move(ros2_event));
                std::cout << "Received Pose { x: " << pose->x << ", y: " << pose->y << " }" << std::endl;
            }
            else
            {
                std::cout << "received unexpected ros2 input" << std::endl;
            }
        }
        else if (is_dora(event))
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
    }

    std::cout << "GOODBYE FROM C++ node (using Rust API)" << std::endl;

    return 0;
}
