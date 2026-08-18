// Service (request/reply) server written against the C++ node API.
//
// Mirrors `examples/service-example/server` (Rust): it reads each request's
// metadata, computes a reply, and passes the incoming metadata straight back
// so the `request_id` correlation reaches the client unchanged.

#include <dora-node-api.h>

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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

        // `event_as_input_with_metadata` (not `event_as_input`) is what makes
        // the server side possible: the reply is only correlatable if we can
        // read the request's metadata and echo it back.
        //
        // Held in an `optional` because `DoraInputWithMetadata` owns a
        // `rust::Box<Metadata>`, which has no default constructor.
        std::optional<DoraInputWithMetadata> maybe_input;
        try
        {
            maybe_input = event_as_input_with_metadata(std::move(event));
        }
        catch (const std::exception &e)
        {
            std::cerr << "[server] failed to read request: " << e.what() << std::endl;
            return -1;
        }
        DoraInputWithMetadata &input = *maybe_input;

        // Skip rather than exit: one client sending a malformed request must
        // not take the server down for every other client. The offending
        // client sees its `recv_service_response` time out.
        if (input.data.size() < 2)
        {
            std::cerr << "[server] request needs two operands, got " << input.data.size()
                      << std::endl;
            continue;
        }

        const std::uint8_t a = input.data[0];
        const std::uint8_t b = input.data[1];
        const std::uint8_t sum = static_cast<std::uint8_t>(a + b);

        std::string request_id;
        try
        {
            request_id = std::string(input.metadata->request_id());
        }
        catch (const std::exception &e)
        {
            // A request without a `request_id` cannot be answered: the client
            // would never match the reply. Skip it rather than emitting an
            // uncorrelated response.
            std::cerr << "[server] dropping request without request_id: " << e.what() << std::endl;
            continue;
        }

        std::cout << "[server] " << request_id << ": " << static_cast<unsigned>(a) << " + "
                  << static_cast<unsigned>(b) << " = " << static_cast<unsigned>(sum) << std::endl;

        std::vector<std::uint8_t> payload{sum};
        rust::Slice<const std::uint8_t> payload_slice{payload.data(), payload.size()};

        // Pass the request's metadata through unchanged — it carries the
        // `request_id` the client is blocked on.
        auto result = send_service_response(
            dora_node.send_output,
            "response",
            payload_slice,
            std::move(input.metadata));
        const std::string error(result.error);
        if (!error.empty())
        {
            std::cerr << "[server] failed to send response: " << error << std::endl;
            return -1;
        }
    }

    return 0;
}
