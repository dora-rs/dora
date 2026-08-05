// A service client that never blocks (dora-rs/dora#3046).
//
// `service-client.cc` in this example is the simple shape: send one
// request, block in `recv_service_response` until it comes back, repeat.
// That is fine when waiting is all the node has to do.
//
// This node is the other shape — the one a single-threaded node driven by
// something other than dora needs:
//
//   * several requests are in flight at once,
//   * the node's own schedule (here a wall clock; in a real node an
//     external transport, a sensor, a control loop) must keep running,
//   * so it can never afford to stop inside a receive.
//
// It uses `try_recv_service_response`, which returns `NotReady` instead
// of waiting, and keeps a small table of outstanding requests. Every
// iteration polls each entry exactly once, so the loop is bounded no
// matter how slow or wedged a server is.
//
// ---------------------------------------------------------------------
// ONE RULE WORTH KNOWING
//
// The correlated polls and `next_event` read the *same* event stream, so
// whichever runs first consumes what is there. A poll correlates the
// reply it wants and buffers everything else for a later `next_event`,
// so polling first never loses anything. The reverse is not true: a
// reply handed to `next_event` has been consumed, and no later poll can
// see it.
//
// This node therefore lets the polls own the stream completely — it has
// no tick input and never calls `next_event`. Shutdown arrives through
// the polls as `StreamEnded`. A node that does need its own dora inputs
// should poll first, then drain with `try_next_event`.
// ---------------------------------------------------------------------

#include <dora-node-api.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{

constexpr const char *SERVER = "cxx-service-server";

// How many requests this demo completes before exiting.
constexpr int REQUEST_BUDGET = 6;

// How long a request may stay outstanding. The framework applies no
// deadline to a poll — that is the trade for not blocking — so the
// client keeps its own, per request.
constexpr auto REQUEST_DEADLINE = std::chrono::seconds(5);

// How many requests may be outstanding at once. This is the property
// under test: a blocking client is structurally limited to one.
constexpr std::size_t MAX_IN_FLIGHT = 3;

// How long the node sleeps between sweeps. Its own choice, which is the
// point — nothing here is dictated by how fast a server replies.
constexpr auto POLL_INTERVAL = std::chrono::milliseconds(10);

using Clock = std::chrono::steady_clock;

struct Pending
{
    std::string request_id;
    std::uint8_t a;
    std::uint8_t b;
    Clock::time_point sent_at;
};

/// Fire one request without waiting for it, returning its correlation id.
///
/// `new_request_id()` + `send_service_request_with_id` rather than
/// `send_service_request`, because the id is needed *before* the reply
/// comes back — it goes straight into the pending table.
std::string send_one(DoraNode &node, std::uint8_t a, std::uint8_t b)
{
    auto request_id = std::string(new_request_id());

    std::vector<std::uint8_t> payload{a, b};
    rust::Slice<const std::uint8_t> slice{payload.data(), payload.size()};

    auto result = send_service_request_with_id(
        node.send_output,
        "request",
        slice,
        new_metadata(),
        request_id);
    const std::string error(result.error);
    if (!error.empty())
    {
        std::cerr << "[polling-client] send failed: " << error << std::endl;
        return {};
    }
    return request_id;
}

/// Outcome of one sweep over the outstanding requests.
struct SweepResult
{
    int completed = 0;
    bool stream_ended = false;
};

/// Poll every outstanding request exactly once.
///
/// Always returns promptly: `NotReady` costs nothing and simply leaves
/// the entry in the table for the next sweep.
SweepResult sweep(DoraNode &node, std::vector<Pending> &pending)
{
    SweepResult out;
    std::vector<Pending> still_waiting;
    still_waiting.reserve(pending.size());
    const auto now = Clock::now();

    for (auto &entry : pending)
    {
        auto result = try_recv_service_response(node.events, entry.request_id, SERVER);

        switch (result.status)
        {
        case DoraPatternStatus::Matched:
        {
            auto input = event_as_input(std::move(result.event));
            const unsigned sum = input.data.empty() ? 0u : static_cast<unsigned>(input.data[0]);
            std::cout << "[polling-client] " << entry.request_id << ": "
                      << static_cast<unsigned>(entry.a) << " + " << static_cast<unsigned>(entry.b)
                      << " = " << sum << std::endl;
            out.completed++;
            break;
        }

        case DoraPatternStatus::NotReady:
            if (now - entry.sent_at > REQUEST_DEADLINE)
            {
                std::cerr << "[polling-client] deadline passed for " << entry.request_id
                          << std::endl;
            }
            else
            {
                still_waiting.push_back(entry);
            }
            break;

        case DoraPatternStatus::StreamEnded:
            // The dataflow is shutting down; stop cleanly.
            out.stream_ended = true;
            break;

        case DoraPatternStatus::ServerRestarted:
            // The in-flight correlation is orphaned; a real client would
            // resend against the new instance.
            std::cerr << "[polling-client] server restarted, dropping " << entry.request_id
                      << std::endl;
            break;

        default:
            std::cerr << "[polling-client] " << entry.request_id
                      << " failed: " << std::string(result.error) << std::endl;
            break;
        }

        if (out.stream_ended)
        {
            // Leave `pending` untouched: entries after this one have not
            // been looked at, so replacing it with the partial
            // `still_waiting` would under-report what is outstanding.
            return out;
        }
    }

    pending = std::move(still_waiting);
    return out;
}

} // namespace

int main()
{
    auto dora_node = init_dora_node();

    std::vector<Pending> pending;
    std::uint8_t next = 0;
    int completed = 0;
    int issued = 0;

    while (completed < REQUEST_BUDGET || !pending.empty())
    {
        // The node's own work, on its own schedule — never gated on a
        // reply. This is what a blocking receive would have stalled: it
        // fills the pipeline to MAX_IN_FLIGHT before looking at any of
        // the answers.
        while (issued < REQUEST_BUDGET && pending.size() < MAX_IN_FLIGHT)
        {
            const auto a = next;
            const auto b = static_cast<std::uint8_t>(next + 10);
            auto request_id = send_one(dora_node, a, b);
            if (request_id.empty())
            {
                break;
            }
            pending.push_back(Pending{request_id, a, b, Clock::now()});
            std::cout << "[polling-client] sent " << request_id << ": "
                      << static_cast<unsigned>(a) << " + " << static_cast<unsigned>(b) << " ("
                      << pending.size() << " in flight)" << std::endl;
            next++;
            issued++;
        }

        auto result = sweep(dora_node, pending);
        completed += result.completed;
        if (result.stream_ended)
        {
            std::cerr << "[polling-client] stream ended with " << pending.size()
                      << " request(s) outstanding" << std::endl;
            return 0;
        }

        std::this_thread::sleep_for(POLL_INTERVAL);
    }

    std::cout << "[polling-client] completed " << completed
              << " request(s), several in flight at a time, never blocking" << std::endl;
    return 0;
}
