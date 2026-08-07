// Memory-pool producer.
//
// Registers two pools with the daemon and drives a fixed script in lockstep
// with the consumer, one round trip per step. Lockstep is what makes the
// consumer's assertions deterministic: the producer never touches a pool
// while a frame the consumer has not yet read is still in it.
//
// `frames` is filled **in place** through `dora::PoolWriteGuard` — a ring of
// slots in one pool, with only the slot index on the topic. `banner` is
// written once through the copy path, `write_memory_pool`. They are separate
// pools because the copy path brackets its own write cycle and so cannot run
// while a guard holds one open.

#include <dora-node-api.h>

#include <dora/memory_pool.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "protocol.h"

namespace
{

using namespace pool_example;

DoraMemoryPoolSpec pool_spec(const char *id, std::size_t size, const std::vector<std::size_t> &shape)
{
    DoraMemoryPoolSpec spec{};
    spec.id = rust::String(id);
    spec.size = size;
    spec.dtype = rust::String("uint8");
    spec.shape = rust::Vec<std::size_t>();
    for (std::size_t dim : shape)
    {
        spec.shape.push_back(dim);
    }
    spec.transport = rust::String("auto");
    spec.receiver_is_cuda = false;
    return spec;
}

void describe(const rust::Box<DoraMemoryPool> &pool)
{
    std::cout << "[sender] pool `" << std::string(pool_id(pool)) << "` transport="
              << std::string(pool_transport(pool)) << " segment="
              << std::string(pool_shm_name(pool)) << " payload=" << pool_payload_len(pool)
              << " bytes" << std::endl;
}

/// Fill one slot of `frames` in place.
///
/// `frame_complete` stands in for what the producer's data source tells it. On
/// false the function returns **without** committing, from inside a live write
/// cycle — the early exit `PoolWriteGuard` exists for. The destructor closes
/// the cycle as incomplete, so the pool's generation is left odd and every
/// reader rejects the half-filled slot instead of taking it for a frame. The
/// next successful write recovers the pool.
bool fill_slot(rust::Box<DoraMemoryPool> &pool, std::uint8_t slot, std::uint8_t value,
               bool frame_complete)
{
    dora::PoolWriteGuard write(pool);

    const std::size_t offset = static_cast<std::size_t>(slot) * kSlotBytes;
    // Bounded by the payload the pool actually has, never by a product
    // computed from the declared shape.
    if (write.size() < offset + kSlotBytes)
    {
        std::cerr << "[sender] slot " << int(slot) << " does not fit in " << write.size()
                  << " payload bytes" << std::endl;
        return false;
    }

    const std::size_t filled = frame_complete ? kSlotBytes : kSlotBytes / 2;
    std::fill_n(write.data() + offset, filled, value);
    if (!frame_complete)
    {
        return false;
    }

    write.commit();
    return true;
}

/// A pool the node still owns, so that every exit path can hand it back to the
/// daemon. `free_memory_pool` consumes the box, which is why this is an
/// `optional` rather than a bare `rust::Box`.
using OwnedPool = std::optional<rust::Box<DoraMemoryPool>>;

bool release(rust::Box<OutputSender> &sender, OwnedPool &pool, const char *label)
{
    if (!pool)
    {
        return true;
    }
    auto result = free_memory_pool(sender, std::move(*pool));
    pool.reset();
    if (!result.error.empty())
    {
        std::cerr << "[sender] failed to free pool `" << label << "`: "
                  << std::string(result.error) << std::endl;
        return false;
    }
    std::cout << "[sender] freed pool `" << label << "`" << std::endl;
    return true;
}

bool send_notice(rust::Box<OutputSender> &sender, std::uint8_t step, std::uint8_t kind,
                 std::uint8_t slot, std::uint8_t value)
{
    const std::uint8_t notice[kNoticeBytes] = {step, kind, slot, value};
    auto result = send_output(sender, "notice", rust::Slice<const std::uint8_t>(notice, kNoticeBytes));
    if (!result.error.empty())
    {
        std::cerr << "[sender] failed to send notice: " << std::string(result.error) << std::endl;
        return false;
    }
    return true;
}

} // namespace

int main()
{
    auto dora_node = init_dora_node();

    OwnedPool frames;
    OwnedPool banner;
    try
    {
        frames = register_memory_pool(dora_node.send_output,
                                      pool_spec(kFramesPool, kFramesBytes, {kSlotCount, kSlotBytes}));
        banner = register_memory_pool(dora_node.send_output,
                                      pool_spec(kBannerPool, kBannerBytes, {kBannerBytes}));
    }
    catch (const std::exception &e)
    {
        std::cerr << "[sender] failed to register a pool: " << e.what() << std::endl;
        return 1;
    }
    describe(*frames);
    describe(*banner);

    // The copy path, on the pool that never sees a write guard. The length
    // must be exactly the payload length: a short copy would leave the tail of
    // the previous frame in place and publish it as part of this one.
    {
        std::vector<std::uint8_t> payload(pool_payload_len(*banner), kBannerByte);
        auto result = write_memory_pool(
            *banner, rust::Slice<const std::uint8_t>(payload.data(), payload.size()));
        if (!result.error.empty())
        {
            std::cerr << "[sender] failed to write `banner`: " << std::string(result.error)
                      << std::endl;
            release(dora_node.send_output, frames, kFramesPool);
            release(dora_node.send_output, banner, kBannerPool);
            return 1;
        }
        std::cout << "[sender] wrote `banner` through the copy path, " << payload.size()
                  << " bytes" << std::endl;
    }

    // The step the consumer has yet to ack. 0 is the hello, which the tick
    // repeats until the consumer is up.
    std::uint8_t step = 0;
    bool failed = false;
    bool finished = false;

    while (!failed && !finished)
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
            std::cerr << "[sender] failed to read input: " << e.what() << std::endl;
            failed = true;
            break;
        }

        const std::string input_id(input.id);
        if (input_id == "tick")
        {
            if (step == 0 && !send_notice(dora_node.send_output, 0, kHello, 0, 0))
            {
                failed = true;
            }
            continue;
        }
        if (input_id != "ack" || input.data.size() < kAckBytes || input.data[0] != step)
        {
            // A repeated hello ack, or an ack for a step already past.
            continue;
        }

        if (step == kFreeStep)
        {
            finished = true;
            break;
        }

        step = static_cast<std::uint8_t>(step + 1);

        std::uint8_t kind = kFrame;
        std::uint8_t slot = 0;
        std::uint8_t value = 0;
        if (step <= kFrameSteps)
        {
            slot = slot_for_step(step);
            value = step;
            failed = !fill_slot(*frames, slot, value, true);
        }
        else if (step == kAbandonStep)
        {
            kind = kAbandoned;
            slot = kAbandonSlot;
            value = kPoisonValue;
            // Expected to report failure: the frame is deliberately abandoned.
            if (fill_slot(*frames, slot, value, false))
            {
                std::cerr << "[sender] the abandoned write reported success" << std::endl;
                failed = true;
            }
            else
            {
                std::cout << "[sender] step " << int(step)
                          << ": abandoned a write guard on slot " << int(slot)
                          << " without committing" << std::endl;
            }
        }
        else if (step == kRecoverStep)
        {
            slot = kAbandonSlot;
            value = kRecoverValue;
            failed = !fill_slot(*frames, slot, value, true);
        }
        else
        {
            kind = kFreed;
            failed = !release(dora_node.send_output, frames, kFramesPool);
        }

        if (!failed && !send_notice(dora_node.send_output, step, kind, slot, value))
        {
            failed = true;
        }
    }

    // On every path, including a Stop that cut the script short: an
    // unreleased pool is a segment the daemon has to sweep instead of unlink.
    failed = !release(dora_node.send_output, frames, kFramesPool) || failed;
    failed = !release(dora_node.send_output, banner, kBannerPool) || failed;

    if (!finished)
    {
        std::cerr << "[sender] stopped at step " << int(step) << " of " << int(kFreeStep)
                  << " without completing the script" << std::endl;
        return 1;
    }
    if (failed)
    {
        return 1;
    }
    std::cout << "[sender] completed all " << int(kFreeStep) << " steps" << std::endl;
    return 0;
}
