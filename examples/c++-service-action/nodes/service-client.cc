// Service (request/reply) client written against the C++ node API.
//
// Mirrors `examples/service-example/client` (Rust): each tick it sends a
// request carrying two operands and blocks until the correlated response
// comes back, using the framework's `request_id` correlation, timeout and
// server-restart fault tolerance rather than hand-rolling any of it.

#include <dora-node-api.h>

#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

// The server node this client correlates against. `recv_service_response`
// needs it so it can abort the wait early if that node restarts, instead
// of blocking on a correlation the restarted server no longer knows about.
constexpr char SERVER_NODE_ID[] = "cxx-service-server";
constexpr std::uint64_t RESPONSE_TIMEOUT_MS = 5000;
constexpr int REQUEST_COUNT = 5;

std::string node_id()
{
    const char *from_env = std::getenv("DORA_NODE_ID");
    return from_env != nullptr ? std::string(from_env) : std::string("cxx-service-client");
}

} // namespace

int main()
{
    const std::string self = node_id();
    auto dora_node = init_dora_node();

    int sent = 0;
    while (sent < REQUEST_COUNT)
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

        DoraInput input;
        try
        {
            input = event_as_input(std::move(event));
        }
        catch (const std::exception &e)
        {
            std::cerr << "[" << self << "] failed to read input: " << e.what() << std::endl;
            return -1;
        }

        // Responses are also routed to this node's `response` input. The
        // blocking `recv_service_response` below normally consumes them, so
        // anything arriving here is an orphan (e.g. a reply that landed after
        // its wait timed out) and is deliberately ignored.
        if (std::string(input.id) != "tick")
        {
            continue;
        }

        const std::uint8_t a = static_cast<std::uint8_t>(sent);
        const std::uint8_t b = static_cast<std::uint8_t>(sent + 10);
        sent += 1;

        std::vector<std::uint8_t> payload{a, b};
        rust::Slice<const std::uint8_t> payload_slice{payload.data(), payload.size()};

        // `send_service_request` generates the `request_id` and injects it
        // into the metadata for us; there is no need to set it by hand.
        auto request = send_service_request(
            dora_node.send_output,
            "request",
            payload_slice,
            new_metadata());
        if (!std::string(request.error).empty())
        {
            std::cerr << "[" << self << "] failed to send request: "
                      << std::string(request.error) << std::endl;
            return -1;
        }

        const std::string request_id(request.request_id);
        std::cout << "[" << self << "] sent request " << request_id << ": "
                  << static_cast<unsigned>(a) << " + " << static_cast<unsigned>(b) << std::endl;

        auto reply = recv_service_response(
            dora_node.events,
            request_id,
            SERVER_NODE_ID,
            RESPONSE_TIMEOUT_MS);

        switch (reply.status)
        {
        case DoraPatternStatus::Matched:
        {
            DoraInput response;
            try
            {
                response = event_as_input(std::move(reply.event));
            }
            catch (const std::exception &e)
            {
                std::cerr << "[" << self << "] failed to read response: " << e.what() << std::endl;
                return -1;
            }
            if (response.data.empty())
            {
                std::cerr << "[" << self << "] response " << request_id << " had no payload"
                          << std::endl;
                return -1;
            }
            const unsigned sum = response.data[0];
            std::cout << "[" << self << "] response " << request_id << ": "
                      << static_cast<unsigned>(a) << " + " << static_cast<unsigned>(b)
                      << " = " << sum << std::endl;
            if (sum != static_cast<unsigned>(static_cast<std::uint8_t>(a + b)))
            {
                std::cerr << "[" << self << "] sum mismatch for " << request_id << std::endl;
                return -1;
            }
            break;
        }
        case DoraPatternStatus::Timeout:
            // Recoverable: the server may just be slow. Drop this request and
            // carry on with the next tick.
            std::cerr << "[" << self << "] request " << request_id << " timed out" << std::endl;
            break;
        case DoraPatternStatus::ServerRestarted:
            // The server lost its correlation state, so this request will
            // never be answered. A production client would re-send it.
            std::cerr << "[" << self << "] server restarted while awaiting " << request_id
                      << "; request must be retried" << std::endl;
            break;
        case DoraPatternStatus::StreamEnded:
            std::cout << "[" << self << "] dataflow stopping" << std::endl;
            return 0;
        default:
            std::cerr << "[" << self << "] request " << request_id
                      << " failed: " << std::string(reply.error) << std::endl;
            return -1;
        }
    }

    std::cout << "[" << self << "] completed " << sent << " request(s)" << std::endl;
    return 0;
}
