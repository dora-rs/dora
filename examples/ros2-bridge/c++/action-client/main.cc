#include "cxx.h"
#include "dora-node-api.h"
#include "ros2-bridge/impl.h"
#include "ros2-bridge/msg/example_interfaces.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
using namespace std::chrono_literals;

int main()
{
    std::cout << "HELLO FROM C++" << std::endl;

    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto service_qos = qos_default();
    service_qos.durability = Ros2Durability::Volatile;
    service_qos.liveliness = Ros2Liveliness::Automatic;
    service_qos.reliable = true;
    service_qos.max_blocking_time = 0.1;
    auto qos = actionqos_default();
    qos.goal_service = service_qos;
    qos.result_service = service_qos;
    qos.cancel_service = service_qos;
    qos.feedback_subscription = service_qos;
    qos.status_subscription = service_qos;

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/dora_ros2_bridge", "action_client");
    auto client = example_interfaces::create_Fibonacci_action_client(
        *node, "/", "fibonacci", qos, merged_events);

    client->wait_for_action(node);
    std::unique_ptr<::rust::Box<::ActionGoalId>> goal_id;
    for(int count = 0; count < 3;) {
        auto event = merged_events.next();
        if (event.is_dora()) { // Event from Dora
            auto dora_event = downcast_dora(std::move(event));
            auto ty = event_type(dora_event);

            if (ty == DoraEventType::Input) {
                auto input = event_as_input(std::move(dora_event));
                if (!goal_id) {
                    auto goal = example_interfaces::Fibonacci_Goal{ .order = 10 + count };
                    goal_id = std::make_unique<::rust::Box<::ActionGoalId>>(
                        client->send_goal(goal));
                    std::cout << "Goal have been sent" << std::endl;
                    client->request_result(*goal_id);
                }
            } else if (ty == DoraEventType::Stop || ty == DoraEventType::AllInputsClosed) {
                std::cout << "Received stop event" << std::endl;
                break;
            }
        } else if (client->matches_result(event)) { // Result Event from ActionClient
            std::cout << "Get result event" << std::endl;

            auto result_event = client->downcast_result(std::move(event));

            std::cout << "Result: ";
            for(auto& num: result_event->get_result().sequence) {
                std::cout << num << " ";
            }
            std::cout << std::endl;

            auto status = result_event->get_status();

            std::cout << "Status: " << static_cast<::std::int16_t>(status) << std::endl;
            if(status == ::ActionStatusEnum::Succeeded) {
                std::cout << "Goal succeeded" << std::endl;
                goal_id = nullptr;
                count += 1;
            } else if(status == ::ActionStatusEnum::Aborted) {
                std::cerr << "Goal aborted" << std::endl;
            } else if(status == ::ActionStatusEnum::Canceled) {
                std::cerr << "Goal canceled" << std::endl;
            }
        } else if (client->matches_status(event)) { // Feedback Event from ActionClient
            std::cout << "Get status event" << std::endl;

            auto status_event = client->downcast_status(std::move(event));

            std::cout << "Status: " << static_cast<::std::int16_t>(status_event->get_status()) << std::endl;
        } else if (client->matches_feedback(event)) { // Status Event from ActionClient
            std::cout << "Get feedback event" << std::endl;

            auto feedback_event = client->downcast_feedback(std::move(event));

            if (!feedback_event->matches_goal(*goal_id)) { // skip if the goal id does not match
                std::cout << "Feedback not belongs to current requesting goal" << std::endl;
                continue;
            }

            if(feedback_event->matches_goal(*goal_id)) {
                auto feedback = feedback_event->get_feedback();
                std::cout << "Feedback: ";
                for(auto& num: feedback.sequence) {
                    std::cout << num << " ";
                }
            }
            std::cout << std::endl;

        }
    }
    std::cout << "GOODBYE FROM C++ node (using Rust API)" << std::endl;
    return 0;
}
