// Memory-pool consumer that reads the pool **from a CUDA kernel**.
//
// This is the acceptance case of dora-rs#2686: a C++ node registers a memory
// pool and does zero-copy CPU->GPU transfer through the host-pinned path on an
// integrated GPU, with no CUDA IPC anywhere.
//
// It runs against the *same, unmodified* producer as the CPU consumer. The
// only difference is `DORA_MEMORY_POOL_TRANSPORT: unified` in the producer's
// `env:` block in `dataflow-cuda.yml`, which flips the `auto` the producer asks
// for from `shmem` to `unified` without touching its code.
//
// What "zero copy" means here, precisely, and how this file shows it:
//
//   * The producer writes bytes into a POSIX shared-memory segment from
//     another process, with plain stores.
//   * This node calls `cudaHostRegister(..., cudaHostRegisterMapped)` on that
//     segment **once per view** and takes its device alias.
//   * A kernel reads the ring through that alias and compares every byte to
//     the model the producer's notices describe. **Nothing copies the payload
//     host->device.** The only `cudaMemcpy` in this file moves the kernel's
//     three-word verdict back, out of a `cudaMalloc`'d buffer that has nothing
//     to do with the pool.
//
// If the alias did not address the same physical pages, the kernel would read
// something other than what the producer wrote and the run would fail.
//
// The other thing this file exists to model is a release ordering that nothing
// else does: `cudaHostUnregister` has to happen before whatever takes the
// segment away — see `CudaMappedView`.

#include <dora-node-api.h>

#include <dora/cuda_pool.hpp>
#include <dora/memory_pool.hpp>

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

#include "checks.h"
#include "protocol.h"

namespace
{

using namespace pool_example;

constexpr const char *kTag = "[cuda-receiver]";

/// What every slot of a pool should hold, in the form a kernel can take: by
/// value, in the launch's 4 KiB parameter space. The *expectation* therefore
/// never travels through memory shared with the pool — only the payload does,
/// and only in one direction.
struct RingModel
{
    std::uint8_t slot[kSlotCount];
};

constexpr unsigned int kNoBadByte = 0xFFFFFFFFu;

/// The kernel's report. `first_bad` is a byte offset into the payload, or
/// `kNoBadByte`.
struct RingVerdict
{
    unsigned int mismatches;
    unsigned int first_bad;
    unsigned int observed;
};

/// Compare `gridDim.x` slots of `slot_bytes` each against `expected`, reading
/// the payload **through the pool's device alias**.
///
/// `observed` is written without a lock and is diagnostic only: it is the byte
/// seen by whichever thread most recently lowered `first_bad`, which under a
/// storm of mismatches need not be the byte `first_bad` finally names. Whether
/// the run passes depends on `mismatches` alone.
__global__ void check_slots(const std::uint8_t *payload, RingModel expected,
                            unsigned int slot_bytes, RingVerdict *verdict)
{
    const unsigned int slot = blockIdx.x;
    const std::uint8_t want = expected.slot[slot];
    const std::uint8_t *begin = payload + static_cast<std::size_t>(slot) * slot_bytes;
    for (unsigned int i = threadIdx.x; i < slot_bytes; i += blockDim.x)
    {
        const std::uint8_t got = begin[i];
        if (got != want)
        {
            atomicAdd(&verdict->mismatches, 1u);
            const unsigned int offset = slot * slot_bytes + i;
            if (atomicMin(&verdict->first_bad, offset) > offset)
            {
                verdict->observed = got;
            }
        }
    }
}

/// A pool view together with the CUDA registration of its segment.
///
/// The pair has an ordering `dora/cuda_pool.hpp` documents and nothing
/// enforces: `cudaHostUnregister` must run **before** anything can take the
/// segment away, and there are two such moments.
///
///   1. The daemon's free notification, drained by `take_freed_pools()`. The
///      caller invokes `release()` from the drain, before the next call that
///      could unlink the segment.
///   2. An ordinary "done with this view" drop at shutdown. Dropping the
///      `rust::Box` unmaps the segment just as an unlink does, so the same
///      unregistration has to precede it.
///
/// The second is what this class buys: the destructor body runs to completion
/// **before** any member is destroyed, so `~CudaMappedView` unregisters and
/// only then does `view_` drop and unmap. Getting that backwards — unmapping
/// the segment while the driver still has the pages pinned — is undefined and
/// silent.
class CudaMappedView
{
  public:
    explicit CudaMappedView(::rust::Box<DoraMemoryPoolView> view)
        : view_(std::move(view)), id_(view_id(view_))
    {
    }

    CudaMappedView(const CudaMappedView &) = delete;
    CudaMappedView &operator=(const CudaMappedView &) = delete;
    CudaMappedView(CudaMappedView &&) = delete;
    CudaMappedView &operator=(CudaMappedView &&) = delete;

    ~CudaMappedView() { release(); }

    /// Page-lock the segment and take the device address of its payload.
    ///
    /// Once per view, never per frame: `cudaHostRegister` walks and pins every
    /// page of the segment, which is orders of magnitude more expensive than a
    /// frame tick.
    bool map()
    {
        std::uint64_t base = 0;
        std::size_t bytes = 0;
        // The mapping base, not the payload start: `cudaHostRegister` needs a
        // page-aligned address and the payload begins at 256 + metadata.
        if (!view_mapping(view_, base, bytes))
        {
            std::cerr << kTag << " pool `" << id_ << "` reports no mapping to register"
                      << std::endl;
            return false;
        }
        cudaError_t error = cudaSuccess;
        void *host = reinterpret_cast<void *>(static_cast<std::uintptr_t>(base));
        if (!dora::cuda::map_pool(host, bytes, mapped_, &error))
        {
            std::cerr << kTag << " map_pool failed on `" << id_
                      << "`: " << cudaGetErrorString(error) << std::endl;
            return false;
        }
        std::size_t offset = 0;
        // A predicate, not a getter. False for an `ipc` or freed pool, and on
        // false `offset` keeps whatever it held — adding that to the device
        // base would land in the segment header instead of the payload, which
        // corrupts silently rather than crashing.
        if (!view_payload_offset(view_, offset))
        {
            std::cerr << kTag << " pool `" << id_ << "` reports no payload in its mapping"
                      << std::endl;
            return false;
        }
        payload_ = static_cast<const std::uint8_t *>(mapped_.device) + offset;
        std::cout << kTag << " registered " << bytes << " bytes of `" << id_ << "`: host="
                  << mapped_.host << " device=" << mapped_.device << ", payload at +" << offset
                  << std::endl;
        return true;
    }

    /// `cudaHostUnregister`, idempotently, so both release points above can
    /// call it and the second is a no-op.
    void release() noexcept
    {
        if (mapped_.host == nullptr)
        {
            return;
        }
        dora::cuda::unmap_pool(mapped_);
        payload_ = nullptr;
        std::cout << kTag << " unregistered `" << id_ << "` before its segment could go away"
                  << std::endl;
    }

    const ::rust::Box<DoraMemoryPoolView> &view() const { return view_; }

    /// The payload's address **in the device alias**, or null once released.
    const std::uint8_t *payload() const { return payload_; }

  private:
    ::rust::Box<DoraMemoryPoolView> view_;
    std::string id_;
    dora::cuda::MappedPool mapped_;
    const std::uint8_t *payload_ = nullptr;
};

/// How one bracketed kernel read ended.
enum class Read
{
    /// The kernel saw exactly the model, and no writer touched the pool while
    /// it ran.
    Matched,
    /// A writer was mid-frame; whatever the kernel computed must be discarded.
    Torn,
    /// The bytes disagreed with the model, or CUDA failed. Fatal.
    Failed,
};

} // namespace

int main()
{
    auto dora_node = init_dora_node();

    // Asserted, not merely reported. `unified` works on a discrete GPU too —
    // it is only slower there than IPC — so a run that quietly happened on one
    // would print the same success line while demonstrating nothing about the
    // case the transport exists for.
    if (!dora::cuda::is_integrated_gpu())
    {
        std::cerr << kTag
                  << " device 0 is not an integrated GPU (or could not be queried); this node "
                     "exists to demonstrate the integrated-GPU path, where cudaIpcGetMemHandle "
                     "is unsupported"
                  << std::endl;
        return 1;
    }

    RingVerdict *device_verdict = nullptr;
    if (cudaMalloc(&device_verdict, sizeof(RingVerdict)) != cudaSuccess)
    {
        std::cerr << kTag << " cudaMalloc failed: " << cudaGetErrorString(cudaGetLastError())
                  << std::endl;
        return 1;
    }

    std::optional<CudaMappedView> frames;
    std::optional<CudaMappedView> banner;
    std::set<std::string> freed;
    RingModel expected = {};
    int verified = 0;
    int torn = 0;
    int unavailable = 0;
    bool failed = false;
    bool done = false;
    std::vector<std::uint8_t> buffer;

    /// Run the kernel over `slots` slots of the pool's payload and hand back
    /// what it found, bracketed by the pool's seqlock.
    ///
    /// `PoolReadGuard` is documented for exactly this shape of consumer: it
    /// samples the generation, the kernel reads the payload where it lies, and
    /// `valid()` says afterwards whether a writer moved underneath. On false
    /// the kernel's verdict is discarded rather than reported — a mismatch
    /// found in a torn frame says nothing about the mapping.
    const auto read_through_device_alias = [&](const CudaMappedView &pool, unsigned int slots,
                                               unsigned int slot_bytes, const RingModel &model,
                                               std::string &detail) -> Read {
        dora::PoolReadGuard read(pool.view());
        if (read.size() != static_cast<std::size_t>(slots) * slot_bytes)
        {
            detail = "payload is " + std::to_string(read.size()) + " bytes, expected " +
                     std::to_string(static_cast<std::size_t>(slots) * slot_bytes);
            return Read::Failed;
        }
        const RingVerdict fresh = {0, kNoBadByte, 0};
        if (cudaMemcpy(device_verdict, &fresh, sizeof fresh, cudaMemcpyHostToDevice) != cudaSuccess)
        {
            detail = std::string("cudaMemcpy of the verdict slot: ") +
                     cudaGetErrorString(cudaGetLastError());
            return Read::Failed;
        }
        // The pool's bytes are read here and nowhere else: `pool.payload()` is
        // the device alias of pages the producer wrote in another process.
        check_slots<<<slots, 256>>>(pool.payload(), model, slot_bytes, device_verdict);
        RingVerdict verdict = {};
        if (cudaMemcpy(&verdict, device_verdict, sizeof verdict, cudaMemcpyDeviceToHost) !=
            cudaSuccess)
        {
            detail = std::string("the kernel did not complete: ") +
                     cudaGetErrorString(cudaGetLastError());
            return Read::Failed;
        }
        // Nothing computed from the payload may be used until this agrees.
        if (!read.valid())
        {
            return Read::Torn;
        }
        if (verdict.mismatches != 0)
        {
            const unsigned int slot = verdict.first_bad / slot_bytes;
            detail = "the kernel found " + std::to_string(verdict.mismatches) +
                     " mismatched bytes; the first is at offset " +
                     std::to_string(verdict.first_bad) + " (slot " + std::to_string(slot) +
                     "), where the host wrote " + std::to_string(int(model.slot[slot]));
            if (verdict.observed != 0 || model.slot[slot] != 0)
            {
                detail += " and the GPU read " + std::to_string(verdict.observed);
            }
            return Read::Failed;
        }
        return Read::Matched;
    };

    /// Drain the daemon's free notifications — the first of the two points at
    /// which a registration has to be released.
    const auto drain_freed = [&]() {
        for (const auto &id : take_freed_pools())
        {
            const std::string name(id);
            std::cout << kTag << " daemon released pool `" << name << "`" << std::endl;
            freed.insert(name);
            // Before anything else touches the view. `take_freed_pools` has
            // already marked it dead, but the segment is still mapped and
            // still pinned, and it is this call that unpins it.
            if (frames && name == kFramesPool)
            {
                frames->release();
            }
            if (banner && name == kBannerPool)
            {
                banner->release();
            }
        }
    };

    while (!failed && !done)
    {
        auto event = next_event(dora_node.events);
        drain_freed();

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
            std::cerr << kTag << " failed to read input: " << e.what() << std::endl;
            failed = true;
            break;
        }
        if (input.data.size() < kNoticeBytes)
        {
            std::cerr << kTag << " notice is " << input.data.size() << " bytes" << std::endl;
            failed = true;
            break;
        }
        const std::uint8_t step = input.data[0];
        const std::uint8_t kind = input.data[1];
        const std::uint8_t slot = input.data[2];
        const std::uint8_t value = input.data[3];

        if (kind != kHello && !frames)
        {
            std::cerr << kTag << " notice kind " << int(kind) << " before the pools were mapped"
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
                frames.emplace(read_memory_pool(dora_node.send_output, kFramesPool));
                banner.emplace(read_memory_pool(dora_node.send_output, kBannerPool));
            }
            catch (const std::exception &e)
            {
                std::cerr << kTag << " failed to map a pool: " << e.what() << std::endl;
                failed = true;
                break;
            }

            // The acceptance criterion of dora-rs#2686, in three predicates.
            const std::string transport(view_transport(frames->view()));
            const bool ipc_present = view_ipc_present(frames->view());
            std::cout << kTag << " transport=" << transport
                      << " ipc_present=" << (ipc_present ? "true" : "false")
                      << " integrated_gpu=true" << std::endl;
            if (transport != "unified")
            {
                std::cerr << kTag << " expected transport `unified`, got `" << transport
                          << "`. The producer asks for `auto` with receiver_is_cuda unset, so "
                             "the dataflow must set DORA_MEMORY_POOL_TRANSPORT=unified in its "
                             "env: block."
                          << std::endl;
                failed = true;
                break;
            }
            if (ipc_present)
            {
                std::cerr << kTag
                          << " the segment carries a CUDA IPC handle; this path must not use one"
                          << std::endl;
                failed = true;
                break;
            }

            const std::string frames_name(view_shm_name(frames->view()));
            const std::string banner_name(view_shm_name(banner->view()));
            if (!segment_name_from_daemon(frames_name, kFramesPool, kTag) ||
                !segment_name_from_daemon(banner_name, kBannerPool, kTag))
            {
                failed = true;
                break;
            }
            if (!frames->map() || !banner->map())
            {
                failed = true;
                break;
            }

            // The copy-path pool, read through its own device alias. One slot
            // of 32 bytes rather than a ring, so the same kernel covers it.
            RingModel banner_model = {};
            banner_model.slot[0] = kBannerByte;
            std::string detail;
            const Read outcome = read_through_device_alias(
                *banner, 1, static_cast<unsigned int>(kBannerBytes), banner_model, detail);
            if (outcome != Read::Matched)
            {
                std::cerr << kTag << " `banner` did not read back on the GPU: "
                          << (detail.empty() ? "a writer was mid-frame" : detail) << std::endl;
                failed = true;
                break;
            }
            std::cout << kTag << " `banner` verified on the GPU, " << kBannerBytes << " bytes"
                      << std::endl;
            break;
        }

        case kFrame:
        {
            if (slot >= kSlotCount)
            {
                std::cerr << kTag << " slot " << int(slot) << " is out of the ring" << std::endl;
                failed = true;
                break;
            }
            expected.slot[slot] = value;

            // The whole ring on every frame, not just the slot just written: a
            // stale or misaligned alias shows up in the fifteen slots the
            // producer did not touch this step.
            std::string detail;
            Read outcome = Read::Torn;
            for (int attempt = 0; attempt < 2 && outcome == Read::Torn; ++attempt)
            {
                outcome = read_through_device_alias(
                    *frames, static_cast<unsigned int>(kSlotCount),
                    static_cast<unsigned int>(kSlotBytes), expected, detail);
            }
            if (outcome == Read::Torn)
            {
                // In lockstep the producer is blocked on our ack, so a second
                // tear is a genuine fault rather than a busy writer.
                std::cerr << kTag << " step " << int(step) << " was overwritten mid-read twice"
                          << std::endl;
                failed = true;
                break;
            }
            if (outcome == Read::Failed)
            {
                std::cerr << kTag << " step " << int(step) << ": " << detail << std::endl;
                failed = true;
                break;
            }
            ++verified;
            break;
        }

        case kAbandoned:
        {
            // The producer left the generation odd, so the seqlock bracket
            // must reject the read no matter what the kernel saw. No retry:
            // odd is the steady state until the next successful write.
            std::string detail;
            const Read outcome =
                read_through_device_alias(*frames, static_cast<unsigned int>(kSlotCount),
                                          static_cast<unsigned int>(kSlotBytes), expected, detail);
            if (outcome != Read::Torn)
            {
                std::cerr << kTag << " the abandoned frame was accepted as "
                          << (outcome == Read::Matched ? "matching" : "a hard failure: " + detail)
                          << ", expected the seqlock to reject it" << std::endl;
                failed = true;
                break;
            }
            ++torn;
            std::cout << kTag << " step " << int(step) << ": slot " << int(slot)
                      << " rejected as incomplete" << std::endl;
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
                drain_freed();
            }
            if (freed.find(kFramesPool) == freed.end())
            {
                std::cerr << kTag << " no free notification for `" << kFramesPool << "` arrived"
                          << std::endl;
                failed = true;
                break;
            }
            // The drain is the only place that could have done this, and it
            // had to before the view could be dropped or the segment unlinked.
            if (frames->payload() != nullptr)
            {
                std::cerr << kTag << " the mapping of `" << kFramesPool
                          << "` is still registered after the free notification" << std::endl;
                failed = true;
                break;
            }
            if (view_is_alive(frames->view()))
            {
                std::cerr << kTag << " the view of `" << kFramesPool << "` is still alive"
                          << std::endl;
                failed = true;
                break;
            }

            const auto outcome = dora::try_read_pool(frames->view(), buffer);
            if (outcome != dora::PoolReadOutcome::Unavailable)
            {
                std::cerr << kTag << " the freed pool read " << outcome_name(outcome)
                          << ", expected Unavailable" << std::endl;
                failed = true;
                break;
            }
            ++unavailable;

            std::size_t offset = 0;
            if (view_payload_offset(frames->view(), offset))
            {
                std::cerr << kTag << " the freed pool still reports a payload offset" << std::endl;
                failed = true;
                break;
            }

            // `banner` is untouched by the free, and still readable through
            // its own device alias — the one mapping that reaches shutdown
            // registered, so the ordinary drop path gets exercised too.
            RingModel banner_model = {};
            banner_model.slot[0] = kBannerByte;
            std::string detail;
            if (read_through_device_alias(*banner, 1, static_cast<unsigned int>(kBannerBytes),
                                          banner_model, detail) != Read::Matched)
            {
                std::cerr << kTag << " `banner` stopped reading after `" << kFramesPool
                          << "` was freed: " << (detail.empty() ? "torn" : detail) << std::endl;
                failed = true;
                break;
            }
            std::cout << kTag << " `" << kFramesPool
                      << "` is permanently unreadable; `banner` still reads on the GPU"
                      << std::endl;
            done = true;
            break;
        }

        default:
            std::cerr << kTag << " unknown notice kind " << int(kind) << std::endl;
            failed = true;
            break;
        }

        if (failed)
        {
            break;
        }

        // Every step is acked, hello and free included. The producer advances
        // on the ack of the step it is waiting for, so a kind this node
        // handled but did not ack stalls the whole script.
        const std::uint8_t ack[kAckBytes] = {step};
        auto result = send_output(dora_node.send_output, "ack",
                                  rust::Slice<const std::uint8_t>(ack, kAckBytes));
        if (!result.error.empty())
        {
            std::cerr << kTag << " failed to ack step " << int(step) << ": "
                      << std::string(result.error) << std::endl;
            failed = true;
        }
    }

    cudaFree(device_verdict);

    std::cout << kTag << " " << verified << " frames verified through the device alias, " << torn
              << " torn, " << unavailable << " unavailable" << std::endl;

    // The second release point, for the pool that was never freed: the
    // destructors unregister and only then drop the views.
    banner.reset();
    frames.reset();

    if (failed || !done || verified != kFrameSteps + 1 || torn != 1 || unavailable != 1)
    {
        std::cerr << kTag << " the run did not cover every outcome" << std::endl;
        return 1;
    }
    return 0;
}
