// Memory-pool consumer.
//
// Maps both of the producer's pools by id — the daemon supplies the segment
// name — and checks, for every step of the producer's script, that the pool
// says what it should:
//
//   * a committed frame reads back whole, through the copy path
//     (`dora::try_read_pool` -> `Copied`) and through the zero-copy path
//     (`dora::PoolReadGuard::valid()`), with every other slot in the ring
//     still holding what it held before;
//   * a frame the producer abandoned mid-write reads back as `Torn` and never
//     as data, and the next committed frame recovers the pool;
//   * a freed pool reads back as `Unavailable` — permanently, not retryably —
//     while the producer's other pool is untouched.
//
// Any of those failing exits non-zero, which fails the dataflow and so the
// example.

#include <dora-node-api.h>

#include <dora/memory_pool.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "protocol.h"

namespace
{

using namespace pool_example;

using OwnedView = std::optional<rust::Box<DoraMemoryPoolView>>;

/// What the ring should contain. Updated on each committed frame, and checked
/// in full on every read — a write that lands in the wrong slot shows up here
/// even though the frame the producer announced is correct.
using Ring = std::uint8_t[kSlotCount];

const char *outcome_name(dora::PoolReadOutcome outcome)
{
    switch (outcome)
    {
    case dora::PoolReadOutcome::Copied:
        return "Copied";
    case dora::PoolReadOutcome::Torn:
        return "Torn";
    case dora::PoolReadOutcome::Unavailable:
        return "Unavailable";
    }
    return "?";
}

/// Empty when every slot of `frame` holds the byte the ring model says it
/// does; otherwise a description of the first byte that disagrees.
///
/// Returned rather than printed, because the zero-copy caller must not report a
/// mismatch before `PoolReadGuard::valid()` has confirmed the bytes were stable
/// at all: a writer overwriting the frame mid-read produces arbitrary
/// differences, and printing them first buries the real cause under a
/// misleading byte-mismatch line.
std::string ring_mismatch(const std::uint8_t *frame, std::size_t len, const Ring &expected)
{
    if (len != kFramesBytes)
    {
        return "payload is " + std::to_string(len) + " bytes, expected " +
               std::to_string(kFramesBytes);
    }
    for (std::size_t slot = 0; slot < kSlotCount; ++slot)
    {
        const std::uint8_t *begin = frame + slot * kSlotBytes;
        const auto *bad =
            std::find_if(begin, begin + kSlotBytes,
                         [&](std::uint8_t byte) { return byte != expected[slot]; });
        if (bad != begin + kSlotBytes)
        {
            return "slot " + std::to_string(slot) + " byte " + std::to_string(bad - begin) +
                   " is " + std::to_string(int(*bad)) + ", expected " +
                   std::to_string(int(expected[slot]));
        }
    }
    return {};
}

/// The segment name can only have come from the daemon.
///
/// This node passed `read_memory_pool` a pool id and nothing else. The name it
/// got back is `dora_pool_<dataflow-uuid>_<owning-node-id>_<pool-id>`: the
/// dataflow uuid is minted per run and the owning node id belongs to the other
/// node, so neither is anything this one could have derived from the id it
/// asked with.
///
/// **This hard-codes the segment grammar on purpose** — it is a contract lock
/// on `libraries/extensions/memory-pool/src/naming.rs::segment_name`, which is
/// what makes "the consumer did not guess the name" checkable at all. A
/// deliberate change to that grammar (or to `DataflowId` no longer being a
/// uuid) will fail here, and this is the reason why.
bool segment_name_from_daemon(const std::string &name, const std::string &pool_id)
{
    const std::string prefix = "dora_pool_";
    const std::string suffix = std::string("_") + kSenderNodeId + "_" + pool_id;
    if (name.size() <= prefix.size() + suffix.size() ||
        name.compare(0, prefix.size(), prefix) != 0 ||
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
        std::cerr << "[receiver] segment name `" << name << "` is not the daemon's name for pool `"
                  << pool_id << "`" << std::endl;
        return false;
    }
    const std::string dataflow_id =
        name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
    if (dataflow_id.size() != 36 || std::count(dataflow_id.begin(), dataflow_id.end(), '-') != 4)
    {
        std::cerr << "[receiver] segment name `" << name << "` carries `" << dataflow_id
                  << "` where the dataflow uuid should be" << std::endl;
        return false;
    }
    return true;
}

/// Drain the daemon's free notifications. This is what marks a view dead, so
/// it has to happen once per event-loop pass and on the same thread that maps
/// pools.
void drain_freed(std::set<std::string> &freed)
{
    for (const auto &id : take_freed_pools())
    {
        const std::string name(id);
        std::cout << "[receiver] daemon released pool `" << name << "`" << std::endl;
        freed.insert(name);
    }
}

} // namespace

int main()
{
    auto dora_node = init_dora_node();

    OwnedView frames;
    OwnedView banner;
    std::set<std::string> freed;
    Ring expected = {};
    int copied = 0;
    int zero_copy = 0;
    int torn = 0;
    int unavailable = 0;
    bool failed = false;
    bool done = false;
    std::vector<std::uint8_t> buffer;

    while (!failed && !done)
    {
        auto event = next_event(dora_node.events);
        drain_freed(freed);

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
            std::cerr << "[receiver] failed to read input: " << e.what() << std::endl;
            failed = true;
            break;
        }
        if (input.data.size() < kNoticeBytes)
        {
            std::cerr << "[receiver] notice is " << input.data.size() << " bytes" << std::endl;
            failed = true;
            break;
        }
        const std::uint8_t step = input.data[0];
        const std::uint8_t kind = input.data[1];
        const std::uint8_t slot = input.data[2];
        const std::uint8_t value = input.data[3];

        if (kind != kHello && !frames)
        {
            std::cerr << "[receiver] notice kind " << int(kind) << " before the pools were mapped"
                      << std::endl;
            failed = true;
            break;
        }

        switch (kind)
        {
        case kHello:
        {
            if (frames)
            {
                break; // A repeated hello; the pools are already mapped.
            }
            try
            {
                frames = read_memory_pool(dora_node.send_output, kFramesPool);
                banner = read_memory_pool(dora_node.send_output, kBannerPool);
            }
            catch (const std::exception &e)
            {
                std::cerr << "[receiver] failed to map a pool: " << e.what() << std::endl;
                failed = true;
                break;
            }
            const auto report = [](const rust::Box<DoraMemoryPoolView> &view, const char *id) {
                const std::string name(view_shm_name(view));
                std::cout << "[receiver] mapped pool `" << id << "` transport="
                          << std::string(view_transport(view)) << " segment=" << name
                          << " payload=" << view_payload_len(view) << " bytes" << std::endl;
                return segment_name_from_daemon(name, id);
            };
            const bool frames_named = report(*frames, kFramesPool);
            const bool banner_named = report(*banner, kBannerPool);
            if (!frames_named || !banner_named)
            {
                failed = true;
                break;
            }
            // The positive half of the liveness check. Without it, a
            // `view_is_alive` stuck at false would satisfy the assertion after
            // the free while having told us nothing.
            if (!view_is_alive(*frames) || !view_is_alive(*banner))
            {
                std::cerr << "[receiver] a freshly mapped view is not alive" << std::endl;
                failed = true;
                break;
            }

            // The copy path's own pool, written once by `write_memory_pool`.
            const auto outcome = dora::try_read_pool(*banner, buffer);
            if (outcome != dora::PoolReadOutcome::Copied)
            {
                std::cerr << "[receiver] `banner` read " << outcome_name(outcome)
                          << ", expected Copied" << std::endl;
                failed = true;
                break;
            }
            if (buffer.size() != kBannerBytes ||
                !std::all_of(buffer.begin(), buffer.end(),
                             [](std::uint8_t byte) { return byte == kBannerByte; }))
            {
                std::cerr << "[receiver] `banner` holds the wrong bytes" << std::endl;
                failed = true;
                break;
            }
            std::cout << "[receiver] `banner` verified, " << buffer.size() << " bytes"
                      << std::endl;
            break;
        }

        case kFrame:
        {
            if (slot >= kSlotCount)
            {
                std::cerr << "[receiver] slot " << int(slot) << " is out of the ring" << std::endl;
                failed = true;
                break;
            }
            expected[slot] = value;

            // Alternating so both reads are exercised over the same ring: the
            // copy on even steps, the in-place read on odd ones.
            if (step % 2 == 0)
            {
                auto outcome = dora::try_read_pool(*frames, buffer);
                if (outcome == dora::PoolReadOutcome::Torn)
                {
                    // `Torn` is retryable, and a real consumer comes back for
                    // the frame on its next event rather than failing. This
                    // harness must terminate, so it retries in place instead —
                    // and in lockstep the producer is blocked on our ack, so a
                    // second `Torn` is a genuine fault, not a busy writer.
                    outcome = dora::try_read_pool(*frames, buffer);
                }
                if (outcome != dora::PoolReadOutcome::Copied)
                {
                    std::cerr << "[receiver] step " << int(step) << " read "
                              << outcome_name(outcome) << " twice, expected Copied" << std::endl;
                    failed = true;
                    break;
                }
                const std::string mismatch =
                    ring_mismatch(buffer.data(), buffer.size(), expected);
                if (!mismatch.empty())
                {
                    std::cerr << "[receiver] step " << int(step) << ": " << mismatch << std::endl;
                    failed = true;
                    break;
                }
                ++copied;
            }
            else
            {
                // Same retry, for the zero-copy read: a guard whose `valid()`
                // is false is the in-place spelling of `Torn`.
                std::string mismatch;
                bool stable = false;
                for (int attempt = 0; attempt < 2 && !stable; ++attempt)
                {
                    dora::PoolReadGuard read(*frames);
                    mismatch = ring_mismatch(read.data(), read.size(), expected);
                    // Everything computed from `data()` is unusable until this
                    // says otherwise — including `mismatch`.
                    stable = read.valid();
                }
                if (!stable)
                {
                    std::cerr << "[receiver] step " << int(step)
                              << " was overwritten mid-read twice" << std::endl;
                    failed = true;
                    break;
                }
                if (!mismatch.empty())
                {
                    std::cerr << "[receiver] step " << int(step) << ": " << mismatch << std::endl;
                    failed = true;
                    break;
                }
                ++zero_copy;
            }
            break;
        }

        case kAbandoned:
        {
            // No retry here, unlike the frame path: an abandoned cycle leaves
            // the generation odd until the next successful write, so `Torn` is
            // the steady state and retrying would only confirm it again.
            const auto outcome = dora::try_read_pool(*frames, buffer);
            if (outcome != dora::PoolReadOutcome::Torn)
            {
                std::cerr << "[receiver] the abandoned frame read " << outcome_name(outcome)
                          << ", expected Torn" << std::endl;
                failed = true;
                break;
            }
            dora::PoolReadGuard read(*frames);
            if (read.valid())
            {
                std::cerr << "[receiver] the abandoned frame passed a zero-copy read"
                          << std::endl;
                failed = true;
                break;
            }
            ++torn;
            std::cout << "[receiver] step " << int(step) << ": slot " << int(slot)
                      << " rejected as incomplete, both reads" << std::endl;
            break;
        }

        case kFreed:
        {
            // The producer freed the pool before sending this, but the
            // daemon's notification is a separate message; wait for it rather
            // than assume it has landed.
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (freed.find(kFramesPool) == freed.end() &&
                   std::chrono::steady_clock::now() < deadline)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                drain_freed(freed);
            }
            if (freed.find(kFramesPool) == freed.end())
            {
                std::cerr << "[receiver] no free notification for `" << kFramesPool
                          << "` arrived" << std::endl;
                failed = true;
                break;
            }
            if (view_is_alive(*frames))
            {
                std::cerr << "[receiver] the view of `" << kFramesPool
                          << "` is still alive after the free" << std::endl;
                failed = true;
                break;
            }

            const auto outcome = dora::try_read_pool(*frames, buffer);
            if (outcome != dora::PoolReadOutcome::Unavailable)
            {
                std::cerr << "[receiver] the freed pool read " << outcome_name(outcome)
                          << ", expected Unavailable" << std::endl;
                failed = true;
                break;
            }
            ++unavailable;

            bool threw = false;
            try
            {
                dora::PoolReadGuard read(*frames);
                (void)read.size();
            }
            catch (const std::exception &)
            {
                threw = true;
            }
            if (!threw)
            {
                std::cerr << "[receiver] a read guard opened on the freed pool" << std::endl;
                failed = true;
                break;
            }

            // Freeing one pool must not disturb the other.
            const auto banner_outcome = dora::try_read_pool(*banner, buffer);
            if (banner_outcome != dora::PoolReadOutcome::Copied)
            {
                std::cerr << "[receiver] `banner` read " << outcome_name(banner_outcome)
                          << " after `frames` was freed, expected Copied" << std::endl;
                failed = true;
                break;
            }
            std::cout << "[receiver] `" << kFramesPool
                      << "` is permanently unreadable; `banner` still reads" << std::endl;
            done = true;
            break;
        }

        default:
            std::cerr << "[receiver] unknown notice kind " << int(kind) << std::endl;
            failed = true;
            break;
        }

        if (failed)
        {
            break;
        }

        const std::uint8_t ack[kAckBytes] = {step};
        auto result =
            send_output(dora_node.send_output, "ack", rust::Slice<const std::uint8_t>(ack, kAckBytes));
        if (!result.error.empty())
        {
            std::cerr << "[receiver] failed to ack step " << int(step) << ": "
                      << std::string(result.error) << std::endl;
            failed = true;
        }
    }

    std::cout << "[receiver] " << copied << " copied reads, " << zero_copy
              << " zero-copy reads, " << torn << " torn, " << unavailable << " unavailable"
              << std::endl;

    // Every outcome of `try_read_pool` has to have been produced and observed;
    // a run that only ever saw `Copied` has verified half the surface.
    if (failed || !done || copied == 0 || zero_copy == 0 || torn != 1 || unavailable != 1)
    {
        std::cerr << "[receiver] the run did not cover every outcome" << std::endl;
        return 1;
    }
    return 0;
}
