// Action (goal / feedback / result) server written against the C++ node API.
//
// Mirrors `examples/action-example/server` (Rust): for each goal it streams
// countdown feedback tagged with the goal's id, then emits one terminal
// result carrying `goal_status`.

#include <dora-node-api.h>

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

// Send one message tagged with `goal_id`, optionally also carrying a
// terminal `goal_status`. Returns false and reports on failure.
bool send_tagged(
    rust::Box<OutputSender> &sender,
    const std::string &output_id,
    const std::string &goal_id,
    const std::string *goal_status,
    std::uint8_t value)
{
    auto metadata = new_metadata();
    metadata->set_goal_id(goal_id);
    if (goal_status != nullptr)
    {
        metadata->set_goal_status(*goal_status);
    }

    std::vector<std::uint8_t> payload{value};
    rust::Slice<const std::uint8_t> payload_slice{payload.data(), payload.size()};

    auto result = send_output_with_metadata(
        sender,
        output_id,
        payload_slice,
        std::move(metadata));
    const std::string error(result.error);
    if (!error.empty())
    {
        std::cerr << "[server] failed to send " << output_id << ": " << error << std::endl;
        return false;
    }
    return true;
}

} // namespace

int main()
{
    auto dora_node = init_dora_node();

    for (;;)
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
            std::cerr << "[server] failed to read goal: " << e.what() << std::endl;
            return -1;
        }
        DoraInputWithMetadata &input = *maybe_input;

        std::string goal_id;
        try
        {
            goal_id = std::string(input.metadata->goal_id());
        }
        catch (const std::exception &e)
        {
            // Without a goal_id the client cannot correlate feedback or the
            // result, so there is nothing useful to emit.
            std::cerr << "[server] dropping goal without goal_id: " << e.what() << std::endl;
            continue;
        }

        const std::uint8_t countdown_from = input.data.empty() ? 0 : input.data[0];
        std::cout << "[server] accepted goal " << goal_id << ": countdown from "
                  << static_cast<unsigned>(countdown_from) << std::endl;

        // Feedback: one message per countdown step, tagged with goal_id but
        // deliberately *without* goal_status, so the client's
        // `recv_action_result` keeps waiting rather than treating these as
        // terminal.
        for (std::uint8_t remaining = countdown_from; remaining > 0; remaining--)
        {
            if (!send_tagged(dora_node.send_output, "feedback", goal_id, nullptr,
                             static_cast<std::uint8_t>(remaining - 1)))
            {
                return -1;
            }
        }

        // Terminal result: goal_id + a terminal goal_status. Use the exported
        // constant rather than a "succeeded" string literal — a typo there
        // would leave the client waiting until its timeout.
        const std::string succeeded(goal_status_succeeded());
        if (!send_tagged(dora_node.send_output, "result", goal_id, &succeeded, 0))
        {
            return -1;
        }

        std::cout << "[server] result " << goal_id << ": " << succeeded << std::endl;
    }

    return 0;
}
