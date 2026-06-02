#include "cxx.h"
#include "dora-node-api.h"
#include "ros2-bridge/impl.h"
#include "ros2-bridge/msg/example_interfaces.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
using namespace std::chrono_literals;

// Concurrent-goal regression test for dora-rs/dora#1972.
//
// Fire several Fibonacci goals up front (so their GetResult round-trips are in
// flight simultaneously) and require a result for every one. Before the fix, the
// generated action client spawned one result receiver per goal, and ros2-client's
// async_request_result discards responses whose request id does not match, so the
// concurrent receivers stole and dropped each other's results — some goals never
// got a result and this node would time out with completed < NUM_GOALS. The fix
// uses a single result pump that demuxes responses by request id, so every goal
// receives its own result.
int main()
{
    const int NUM_GOALS = 3;

    std::cout << "HELLO FROM C++ (concurrent action client)" << std::endl;

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
    auto node = ros2_context->new_node("/dora_ros2_bridge", "action_client_concurrent");
    auto client = example_interfaces::create_Fibonacci_action_client(
        *node, "/", "fibonacci", qos, merged_events);

    client->wait_for_action(node);

    std::vector<::rust::Box<::ActionGoalId>> goals;
    std::vector<bool> done(NUM_GOALS, false);
    bool sent = false;
    int completed = 0;

    // Fail fast instead of hanging if some goal never gets a result (the #1972
    // failure mode). The dora timer keeps this loop alive at 2 Hz, so the
    // deadline is always checked.
    auto deadline = std::chrono::steady_clock::now() + 60s;

    while (completed < NUM_GOALS)
    {
        if (std::chrono::steady_clock::now() >= deadline)
        {
            std::cerr << "TIMEOUT: only " << completed << " of " << NUM_GOALS
                      << " concurrent results received (response stealing? see #1972)"
                      << std::endl;
            return 1;
        }

        auto event = merged_events.next();
        if (event.is_dora())
        {
            auto dora_event = downcast_dora(std::move(event));
            auto ty = event_type(dora_event);
            if (ty == DoraEventType::Input)
            {
                if (!sent)
                {
                    // Send every goal before awaiting any result, so the result
                    // round-trips overlap. Different orders => different durations.
                    for (int i = 0; i < NUM_GOALS; i++)
                    {
                        auto goal = example_interfaces::Fibonacci_Goal{.order = 4 + i};
                        auto gid = client->send_goal(goal);
                        client->request_result(gid);
                        goals.push_back(std::move(gid));
                    }
                    sent = true;
                    std::cout << "Sent " << NUM_GOALS << " concurrent goals" << std::endl;
                }
            }
            else if (ty == DoraEventType::Stop || ty == DoraEventType::AllInputsClosed)
            {
                std::cout << "Received stop event" << std::endl;
                break;
            }
        }
        else if (client->matches_result(event))
        {
            auto result_event = client->downcast_result(std::move(event));
            // Attribute the result to its goal; ignore duplicates and unknowns.
            for (int i = 0; i < NUM_GOALS; i++)
            {
                if (!done[i] && result_event->matches_goal(goals[i]))
                {
                    if (result_event->get_status() == ::ActionStatusEnum::Succeeded)
                    {
                        done[i] = true;
                        completed += 1;
                        std::cout << "Result for goal " << i << " received ("
                                  << completed << "/" << NUM_GOALS << "): ";
                        for (auto &num : result_event->get_result().sequence)
                        {
                            std::cout << num << " ";
                        }
                        std::cout << std::endl;
                    }
                    break;
                }
            }
        }
        // feedback / status events are not needed for this test
    }

    if (completed == NUM_GOALS)
    {
        std::cout << "ALL " << NUM_GOALS << " CONCURRENT RESULTS RECEIVED" << std::endl;
        std::cout << "GOODBYE FROM C++ node (concurrent action client)" << std::endl;
        return 0;
    }
    std::cerr << "FAILED: only " << completed << "/" << NUM_GOALS << " results" << std::endl;
    return 1;
}
