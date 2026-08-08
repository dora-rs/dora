// Action (goal / feedback / result) client written against the C++ node API.
//
// Mirrors `examples/action-example/client` (Rust): each tick it sends a goal
// and blocks until the *terminal* result for that `goal_id` arrives. The
// intermediate feedback messages that arrive during the wait are buffered by
// the framework and replayed by later `next_event` calls, so the main event
// loop below still observes every one of them.

#include <dora-node-api.h>

#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr char SERVER_NODE_ID[] = "cxx-action-server";
constexpr std::uint64_t RESULT_TIMEOUT_MS = 5000;
constexpr int GOAL_COUNT = 3;

std::string node_id()
{
    const char *from_env = std::getenv("DORA_NODE_ID");
    return from_env != nullptr ? std::string(from_env) : std::string("cxx-action-client");
}

} // namespace

int main()
{
    const std::string self = node_id();
    auto dora_node = init_dora_node();

    int sent = 0;
    while (sent < GOAL_COUNT)
    {
        auto event = next_event(dora_node.events);
        const auto ty = event_type(event);

        if (ty == DoraEventType::Stop || ty == DoraEventType::AllInputsClosed)
        {
            break;
        }
        if (ty != DoraEventType::Input)
        {
            continue;
        }

        // `optional` because `DoraInputWithMetadata` owns a
        // `rust::Box<Metadata>`, which has no default constructor.
        std::optional<DoraInputWithMetadata> maybe_input;
        try
        {
            maybe_input = event_as_input_with_metadata(std::move(event));
        }
        catch (const std::exception &e)
        {
            std::cerr << "[" << self << "] failed to read input: " << e.what() << std::endl;
            return -1;
        }
        DoraInputWithMetadata &input = *maybe_input;

        const std::string input_id(input.id);

        // Feedback buffered during a previous `recv_action_result` wait is
        // replayed here — the pattern helper never swallows it.
        if (input_id == "feedback")
        {
            try
            {
                const std::string goal(input.metadata->goal_id());
                const unsigned remaining = input.data.empty() ? 0u : input.data[0];
                std::cout << "[" << self << "] feedback " << goal << ": " << remaining
                          << std::endl;
            }
            catch (const std::exception &e)
            {
                std::cerr << "[" << self << "] feedback without goal_id: " << e.what()
                          << std::endl;
            }
            continue;
        }
        if (input_id != "tick")
        {
            continue;
        }

        // Unlike a service request, the action goal id is generated
        // explicitly and set on the metadata, because the same id also
        // labels the feedback stream.
        const std::string goal_id(new_goal_id());
        auto metadata = new_metadata();
        metadata->set_goal_id(goal_id);

        const std::uint8_t countdown_from = static_cast<std::uint8_t>(3 + sent);
        sent += 1;

        std::vector<std::uint8_t> payload{countdown_from};
        rust::Slice<const std::uint8_t> payload_slice{payload.data(), payload.size()};

        auto send_result = send_output_with_metadata(
            dora_node.send_output,
            "goal",
            payload_slice,
            std::move(metadata));
        const std::string send_error(send_result.error);
        if (!send_error.empty())
        {
            std::cerr << "[" << self << "] failed to send goal: " << send_error << std::endl;
            return -1;
        }

        std::cout << "[" << self << "] sent goal " << goal_id << ": countdown from "
                  << static_cast<unsigned>(countdown_from) << std::endl;

        // Returns only on a terminal status (succeeded / aborted / canceled);
        // feedback carrying the same goal_id does not end the wait.
        auto outcome = recv_action_result(
            dora_node.events,
            goal_id,
            SERVER_NODE_ID,
            RESULT_TIMEOUT_MS);

        switch (outcome.status)
        {
        case DoraPatternStatus::Matched:
        {
            try
            {
                auto result = event_as_input_with_metadata(std::move(outcome.event));
                std::cout << "[" << self << "] result " << goal_id << ": "
                          << std::string(result.metadata->goal_status()) << std::endl;
            }
            catch (const std::exception &e)
            {
                std::cerr << "[" << self << "] failed to read result: " << e.what() << std::endl;
                return -1;
            }
            break;
        }
        case DoraPatternStatus::Timeout:
            std::cerr << "[" << self << "] goal " << goal_id << " timed out" << std::endl;
            break;
        case DoraPatternStatus::ServerRestarted:
            std::cerr << "[" << self << "] server restarted while awaiting goal " << goal_id
                      << "; goal must be re-sent" << std::endl;
            break;
        case DoraPatternStatus::StreamEnded:
            std::cout << "[" << self << "] dataflow stopping" << std::endl;
            return 0;
        default:
            std::cerr << "[" << self << "] goal " << goal_id
                      << " failed: " << std::string(outcome.error) << std::endl;
            return -1;
        }
    }

    std::cout << "[" << self << "] completed " << sent << " goal(s)" << std::endl;
    return 0;
}
