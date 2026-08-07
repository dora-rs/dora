// The two nodes' shared vocabulary: pool geometry, pool ids, and the tiny
// notice/ack messages they exchange on the topic.
//
// The topic carries four bytes. Everything that matters — the frame — travels
// through the pool, which is the whole point of the transport.

#pragma once

#include <cstddef>
#include <cstdint>

namespace pool_example
{

/// `frames`: a ring of 16 slots of 1 KiB, filled in place through
/// `dora::PoolWriteGuard`. One pool, one write cycle per frame, one slot
/// index on the wire — the shape a camera pipeline uses.
constexpr std::size_t kSlotBytes = 1024;
constexpr std::size_t kSlotCount = 16;
constexpr std::size_t kFramesBytes = kSlotBytes * kSlotCount;
constexpr const char *kFramesPool = "frames";

/// `banner`: a second, tiny pool written once through the copy path,
/// `write_memory_pool`. It is a separate pool on purpose — the copy path
/// brackets its own write cycle, so it cannot be used on a pool a
/// `PoolWriteGuard` is holding open.
constexpr std::size_t kBannerBytes = 32;
constexpr std::uint8_t kBannerByte = 0xA5;
constexpr const char *kBannerPool = "banner";

/// The producer's node id, which is also the middle component of the segment
/// name the daemon reports for its pools. The consumer knows it from the
/// dataflow, never from anything it computed — see `segment_name_from_daemon`
/// in the receiver.
constexpr const char *kSenderNodeId = "cxx-pool-sender";

/// What the producer did to `frames` before publishing the notice.
enum Kind : std::uint8_t
{
    /// No pool access. Repeated on every tick until the consumer has mapped
    /// the pools, so the handshake does not depend on which node starts first.
    kHello = 0,
    /// A slot was filled in place and committed.
    kFrame = 1,
    /// A write guard was taken and abandoned without `commit()`.
    kAbandoned = 2,
    /// `frames` has been freed; `banner` is still mapped.
    kFreed = 3,
};

/// Producer -> consumer: `{step, kind, slot, value}`.
constexpr std::size_t kNoticeBytes = 4;
/// Consumer -> producer: `{step}`. Echoed so the producer advances on the ack
/// of the step it is waiting for and never on a repeated hello.
constexpr std::size_t kAckBytes = 1;

/// The script. Steps 1..=20 fill the ring, wrapping it once; then one
/// abandoned write, then a successful write to the slot the abandoned one
/// half-filled, then the free.
constexpr std::uint8_t kFrameSteps = 20;
constexpr std::uint8_t kAbandonStep = kFrameSteps + 1;
constexpr std::uint8_t kRecoverStep = kFrameSteps + 2;
constexpr std::uint8_t kFreeStep = kFrameSteps + 3;

/// Slot 5 is written normally at step 6, abandoned at step 21 and rewritten at
/// step 22 — so the consumer sees the same slot go good, incomplete, good.
constexpr std::uint8_t kAbandonSlot = 5;
constexpr std::uint8_t kPoisonValue = 0xEE;
constexpr std::uint8_t kRecoverValue = 0xC3;

/// The slot a normal frame step writes, and the byte it fills it with.
constexpr std::uint8_t slot_for_step(std::uint8_t step)
{
    return static_cast<std::uint8_t>((step - 1) % kSlotCount);
}

} // namespace pool_example
