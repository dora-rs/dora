#include "../build/dora-node-api.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

// Sends output from worker threads via SafeOutputSender while the main loop
// keeps draining events. create_safe_output_sender consumes send_output, so
// the original sender is unusable afterward.
int main()
{
    std::cout << "HELLO FROM C++ (thread-safe output)" << std::endl;

    auto dora_node = init_dora_node();
    auto safe_sender = create_safe_output_sender(std::move(dora_node.send_output));

    std::vector<std::thread> workers;

    for (int i = 0; i < 20; i++)
    {
        auto event = dora_node.events->next();
        auto ty = event_type(event);

        if (ty == DoraEventType::AllInputsClosed || ty == DoraEventType::Stop)
        {
            break;
        }
        else if (ty == DoraEventType::Input)
        {
            auto input = event_as_input(std::move(event));
            std::string id(input.id);
            std::cout << "[main] received input '" << id
                      << "' -> dispatching to worker " << i << std::endl;

            workers.emplace_back([&safe_sender, i]()
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                std::vector<uint8_t> out_vec{static_cast<uint8_t>(i)};
                rust::Slice<const uint8_t> out_slice{out_vec.data(), out_vec.size()};
                auto result = safe_send_output(*safe_sender, "result", out_slice, new_metadata());

                auto error = std::string(result.error);
                if (!error.empty())
                {
                    std::cerr << "[worker " << i << "] send error: " << error << std::endl;
                }
                else
                {
                    std::cout << "[worker " << i << "] sent result from worker thread" << std::endl;
                }
            });
        }
    }

    for (auto &w : workers)
    {
        w.join();
    }

    std::cout << "GOODBYE FROM C++ (thread-safe output)" << std::endl;
    return 0;
}
