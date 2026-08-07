// Memory-pool consumer for a pool registered by the **Python** binding.
//
// The claim this file tests is that `unified` is not a private C++ convention:
// what `register_memory_pool(tensor_info, "cpu")` writes in Python and what the
// C++ producer writes for a CUDA receiver are the same segment layout, so a C++
// node can map a Python node's pool — on the CPU and, on an integrated GPU,
// from a kernel with no CUDA IPC anywhere.
//
// One source, two binaries. Compiled by g++ it reads the payload on the CPU;
// compiled by nvcc (`-x cu`) it page-locks the segment once and reads the same
// payload through its device alias.
//
// Be precise about what that shares. The two builds differ in exactly two
// places — `read_ring`'s body, and `map()`/`release()`, which are no-ops on the
// CPU build — so the *read* is genuinely two implementations. What the one
// source locks together is everything either build says about the **segment**:
// the pool id arriving as data, `segment_name_matches_python_pool_id`, the
// transport/`ipc_present`/`payload_len` assertions, the ring model and the
// script it is driven by, and the free path down to `try_read_pool` returning
// `Unavailable`. Those cannot disagree between CPU and GPU because there is
// only one copy of them. The CUDA build also never exercises `try_read_pool`'s
// `Copied` path against a Python pool (`use_copy_path` is discarded there,
// deliberately — copying the payload would defeat the point), so that half is
// CPU-only coverage.
//
// # Why this is not `pool-receiver.cc` with a different producer
//
// The C++ example's consumers cannot be reused here, and not for cosmetic
// reasons:
//
//   * A Python pool id is **generated** (`pool_<node-id>_<counter>`), never
//     chosen, so `read_memory_pool(..., "frames")` has nothing to ask for. The
//     id has to travel on a topic.
//   * There is no way to abandon a write cycle through the Python API, so the
//     `Torn` half of `pool-receiver.cc`'s script — which its exit gate requires
//     to have happened exactly once — cannot be produced at all.
//   * A Python CPU pool reports transport `shmem`, not `unified`: Python writes
//     no `transport` key and the C++ side resolves it from `ipc_flag = 0`.
//     `pool-receiver-cuda.cc` asserts `unified` and would fail on a segment
//     that is byte-identical everywhere it matters.
//
// The third point is the interesting one, and this file states it as an
// assertion rather than a footnote. Note what it is *not* saying: the two
// segments are not byte-identical. Dumped side by side for the same 16 KiB
// payload, a Python CPU pool and a C++ `unified` pool disagree in four places —
// Python writes no `transport` key, writes `pinned_type: "cpu"` where C++
// writes `"cuda"`, uses `json.dumps`'s spaced separators against serde_json's
// compact ones (so `json_len` is 73 against 91 even where every key and value
// agrees), and arrives at `write_gen = 2` because its register copies the
// source tensor and closes a seqlock cycle, where `PoolSegment::create` leaves
// `0`.
//
// None of that reaches the consumer, and the reason is the point: `json_len`,
// `data_offset`, `pinned_type` and the generation are all read **out of the
// header**, never assumed. What has to match is the layout the header
// describes — magic, the `json_len`/`data_offset` pair, the seqlock at offset
// 96, and a data region at `data_offset` — and that is what makes the CUDA
// build below able to map a `shmem` pool exactly as it maps a `unified` one.

#include <dora-node-api.h>

#include <dora/memory_pool.hpp>

#ifdef __CUDACC__
#include <dora/cuda_pool.hpp>
#endif

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

#include "checks.h"
#include "protocol.h"

namespace
{

using namespace pool_example;

#ifdef __CUDACC__
constexpr const char *kTag = "[cuda-interop]";
#else
constexpr const char *kTag = "[interop]";
#endif

/// The Python node's id in `dataflow-python-interop.yml`. It is the middle
/// component of both the pool id the sender publishes and the segment name the
/// daemon reports, and the consumer checks the two against each other.
constexpr const char *kPythonSenderNodeId = "python-pool-sender";

/// The script `python_sender.py` runs: 20 frames, wrapping the 16-slot ring
/// once, then the free.
constexpr std::uint8_t kInteropFrameSteps = 20;
constexpr std::uint8_t kInteropFreeStep = kInteropFrameSteps + 1;

/// What every slot of the ring should hold. By value so the CUDA build can put
/// it in the kernel's parameter space: the *expectation* then never travels
/// through memory shared with the pool, only the payload does.
struct RingModel
{
    std::uint8_t slot[kSlotCount];
};

/// How one bracketed read ended.
enum class Read
{
    /// The payload was exactly the model, and no writer touched it meanwhile.
    Matched,
    /// A writer was mid-frame; whatever was computed must be discarded.
    Torn,
    /// The bytes disagreed with the model, or the read machinery failed. Fatal.
    Failed,
};

#ifdef __CUDACC__

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
/// `observed` is written without a lock and is diagnostic only. Whether the run
/// passes depends on `mismatches` alone.
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

#endif // __CUDACC__

/// A view of the Python node's pool, plus — on the CUDA build — the CUDA
/// registration of its segment.
///
/// The registration has an ordering `dora/cuda_pool.hpp` documents and nothing
/// enforces: `cudaHostUnregister` must run before anything can take the segment
/// away, and there are two such moments — the daemon's free notification, and
/// dropping the view at shutdown. The destructor body runs to completion before
/// any member is destroyed, so `release()` there still precedes `view_`'s drop.
class InteropView
{
  public:
    explicit InteropView(::rust::Box<DoraMemoryPoolView> view)
        : view_(std::move(view)), id_(view_id(view_))
    {
    }

    InteropView(const InteropView &) = delete;
    InteropView &operator=(const InteropView &) = delete;
    InteropView(InteropView &&) = delete;
    InteropView &operator=(InteropView &&) = delete;

    ~InteropView() { release(); }

    const ::rust::Box<DoraMemoryPoolView> &view() const { return view_; }

#ifdef __CUDACC__
    /// Page-lock the segment once and take the device address of its payload.
    /// Never per frame: `cudaHostRegister` walks and pins every page.
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
        // A predicate, not a getter. On false `offset` keeps whatever it held,
        // and adding that to the device base lands in the DORADMA header.
        if (!view_payload_offset(view_, offset))
        {
            std::cerr << kTag << " pool `" << id_ << "` reports no payload in its mapping"
                      << std::endl;
            return false;
        }
        payload_ = static_cast<const std::uint8_t *>(mapped_.device) + offset;
        std::cout << kTag << " page-locked " << bytes << " bytes of Python's segment: host="
                  << mapped_.host << " device=" << mapped_.device << ", payload at +" << offset
                  << std::endl;
        return true;
    }

    /// `cudaHostUnregister`, idempotently, so both release points can call it.
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

    /// The payload's address **in the device alias**, or null once released.
    const std::uint8_t *payload() const { return payload_; }
#else
    bool map() { return true; }
    void release() noexcept {}
#endif

  private:
    ::rust::Box<DoraMemoryPoolView> view_;
    std::string id_;
#ifdef __CUDACC__
    dora::cuda::MappedPool mapped_;
    const std::uint8_t *payload_ = nullptr;
#endif
};

/// The segment name can only have come from the daemon.
///
/// The consumer asked with a pool id it read off a topic and nothing else. The
/// uuid/prefix/suffix grammar is checked by `checks.h`, shared with the two C++
/// receivers; all this adds is the Python-specific step of turning the pool id
/// into the tail the daemon's name is expected to end with — Python formats the
/// two strings independently, so the pool id is not a substring of the segment
/// name the way the C++ producer's is.
bool segment_name_matches_python_pool_id(const std::string &name, const std::string &pool_id)
{
    const std::string id_prefix = "pool_";
    if (pool_id.compare(0, id_prefix.size(), id_prefix) != 0)
    {
        std::cerr << kTag << " pool id `" << pool_id << "` is not a Python pool id" << std::endl;
        return false;
    }
    // `<node-id>_<counter>`: the one component the two independently-formatted
    // strings do share.
    const std::string tail = pool_id.substr(id_prefix.size());
    if (tail.compare(0, std::string(kPythonSenderNodeId).size(), kPythonSenderNodeId) != 0)
    {
        std::cerr << kTag << " pool id `" << pool_id << "` does not name `" << kPythonSenderNodeId
                  << "`" << std::endl;
        return false;
    }
    return segment_name_from_daemon(name, tail, pool_id, kTag);
}

} // namespace

int main()
{
    auto dora_node = init_dora_node();

#ifdef __CUDACC__
    // Asserted, not merely reported. The point of this binary is the
    // integrated-GPU path, where `cudaIpcGetMemHandle` is unsupported and the
    // host-pinned mapping is the only zero-copy route.
    if (!dora::cuda::is_integrated_gpu())
    {
        std::cerr << kTag << " device 0 is not an integrated GPU (or could not be queried)"
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
#endif

    std::string pool_id;
    std::optional<InteropView> frames;
    std::set<std::string> freed;
    RingModel expected = {};
    int verified = 0;
    int unavailable = 0;
#ifndef __CUDACC__
    int copied = 0;
    int zero_copy = 0;
#endif
    bool failed = false;
    bool done = false;
    std::vector<std::uint8_t> buffer;

    /// Compare the whole ring against `model`, bracketed by the pool's seqlock.
    ///
    /// `use_copy_path` picks between `try_read_pool` and an in-place read on the
    /// CPU build; the CUDA build always reads in place, through the device
    /// alias, because copying the payload out would defeat the point.
    const auto read_ring = [&](const InteropView &pool, unsigned int slots,
                               unsigned int slot_bytes, const RingModel &model, bool use_copy_path,
                               std::string &detail) -> Read {
        const std::size_t want_bytes = static_cast<std::size_t>(slots) * slot_bytes;

#ifdef __CUDACC__
        (void)use_copy_path;
        dora::PoolReadGuard read(pool.view());
        if (read.size() != want_bytes)
        {
            detail = "payload is " + std::to_string(read.size()) + " bytes, expected " +
                     std::to_string(want_bytes);
            return Read::Failed;
        }
        const RingVerdict fresh = {0, kNoBadByte, 0};
        if (cudaMemcpy(device_verdict, &fresh, sizeof fresh, cudaMemcpyHostToDevice) != cudaSuccess)
        {
            detail = std::string("cudaMemcpy of the verdict slot: ") +
                     cudaGetErrorString(cudaGetLastError());
            return Read::Failed;
        }
        // Python's bytes are read here and nowhere else: `pool.payload()` is the
        // device alias of pages another process wrote with plain stores.
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
                     "), where Python wrote " + std::to_string(int(model.slot[slot])) +
                     " and the GPU read " + std::to_string(verdict.observed);
            return Read::Failed;
        }
        return Read::Matched;
#else
        const auto compare = [&](const std::uint8_t *payload, std::size_t len) -> std::string {
            if (len != want_bytes)
            {
                return "payload is " + std::to_string(len) + " bytes, expected " +
                       std::to_string(want_bytes);
            }
            for (unsigned int slot = 0; slot < slots; ++slot)
            {
                const std::uint8_t *begin = payload + static_cast<std::size_t>(slot) * slot_bytes;
                const auto *bad = std::find_if(begin, begin + slot_bytes,
                                               [&](std::uint8_t byte)
                                               { return byte != model.slot[slot]; });
                if (bad != begin + slot_bytes)
                {
                    return "slot " + std::to_string(slot) + " byte " + std::to_string(bad - begin) +
                           " is " + std::to_string(int(*bad)) + ", expected " +
                           std::to_string(int(model.slot[slot]));
                }
            }
            return {};
        };

        if (use_copy_path)
        {
            const auto outcome = dora::try_read_pool(pool.view(), buffer);
            if (outcome == dora::PoolReadOutcome::Torn)
            {
                return Read::Torn;
            }
            if (outcome != dora::PoolReadOutcome::Copied)
            {
                detail = std::string("read ") + outcome_name(outcome) + ", expected Copied";
                return Read::Failed;
            }
            detail = compare(buffer.data(), buffer.size());
            if (!detail.empty())
            {
                return Read::Failed;
            }
            ++copied;
            return Read::Matched;
        }
        dora::PoolReadGuard read(pool.view());
        const std::string mismatch = compare(read.data(), read.size());
        // Everything computed from `data()` is unusable until this says
        // otherwise — including `mismatch`.
        if (!read.valid())
        {
            return Read::Torn;
        }
        if (!mismatch.empty())
        {
            detail = mismatch;
            return Read::Failed;
        }
        ++zero_copy;
        return Read::Matched;
#endif
    };

    /// Drain the daemon's free notifications. This is what marks a view dead,
    /// and — on the CUDA build — the first of the two points at which the
    /// registration has to be released.
    const auto drain_freed = [&]() {
        for (const auto &id : take_freed_pools())
        {
            const std::string name(id);
            std::cout << kTag << " daemon released pool `" << name << "`" << std::endl;
            freed.insert(name);
            if (frames && name == pool_id)
            {
                frames->release();
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

        // The pool id is data, not a constant. It is generated by the Python
        // binding, so this is the only place the consumer could learn it.
        if (std::string(input.id) == "pool_id")
        {
            if (pool_id.empty())
            {
                pool_id.assign(input.data.begin(), input.data.end());
                std::cout << kTag << " Python published pool id `" << pool_id << "`" << std::endl;
            }
            continue;
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
            std::cerr << kTag << " notice kind " << int(kind) << " before the pool was mapped"
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
                break; // A repeated hello; the pool is already mapped.
            }
            if (pool_id.empty())
            {
                // The id has not landed yet. Not acking is the whole response:
                // the sender re-hellos on its next tick.
                continue;
            }
            try
            {
                frames.emplace(read_memory_pool(dora_node.send_output, pool_id.c_str()));
            }
            catch (const std::exception &e)
            {
                std::cerr << kTag << " failed to map Python's pool: " << e.what() << std::endl;
                failed = true;
                break;
            }

            const std::string name(view_shm_name(frames->view()));
            const std::string transport(view_transport(frames->view()));
            const bool ipc_present = view_ipc_present(frames->view());
            const std::size_t payload_len = view_payload_len(frames->view());
            std::cout << kTag << " mapped Python's pool: segment=" << name
                      << " transport=" << transport
                      << " ipc_present=" << (ipc_present ? "true" : "false")
                      << " payload=" << payload_len << " bytes" << std::endl;

            if (!segment_name_matches_python_pool_id(name, pool_id))
            {
                failed = true;
                break;
            }
            // `shmem`, not `unified`: Python writes no `transport` key and the
            // C++ side resolves it from `ipc_flag = 0`. One of the four ways a
            // Python CPU segment differs from a C++ `unified` one (see the file
            // comment) — and, like the other three, not something the reads
            // below depend on, because they take their offsets and their
            // generation from the header rather than from this label.
            if (transport != "shmem")
            {
                std::cerr << kTag << " expected transport `shmem` for a Python CPU pool, got `"
                          << transport << "`" << std::endl;
                failed = true;
                break;
            }
            if (ipc_present)
            {
                std::cerr << kTag << " the segment carries a CUDA IPC handle; a Python CPU pool "
                             "must not have one"
                          << std::endl;
                failed = true;
                break;
            }
            if (payload_len != kFramesBytes)
            {
                std::cerr << kTag << " payload is " << payload_len << " bytes, expected "
                          << kFramesBytes << std::endl;
                failed = true;
                break;
            }
            if (!view_is_alive(frames->view()))
            {
                std::cerr << kTag << " a freshly mapped view is not alive" << std::endl;
                failed = true;
                break;
            }
            if (!frames->map())
            {
                failed = true;
                break;
            }
            break;
        }

        case kFrame:
        {
            if (step > kInteropFrameSteps || slot >= kSlotCount)
            {
                std::cerr << kTag << " step " << int(step) << " slot " << int(slot)
                          << " is outside the script" << std::endl;
                failed = true;
                break;
            }
            expected.slot[slot] = value;

            // The whole ring on every frame, not just the slot just written: a
            // stale or misaligned mapping shows up in the fifteen slots Python
            // did not touch this step. On the CPU build the copy path and the
            // in-place path alternate, so both are exercised over the same ring.
            std::string detail;
            Read outcome = Read::Torn;
            for (int attempt = 0; attempt < 2 && outcome == Read::Torn; ++attempt)
            {
                outcome = read_ring(*frames, static_cast<unsigned int>(kSlotCount),
                                    static_cast<unsigned int>(kSlotBytes), expected, step % 2 == 0,
                                    detail);
            }
            if (outcome == Read::Torn)
            {
                // In lockstep the sender is blocked on our ack, so a second
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

        case kFreed:
        {
            if (step != kInteropFreeStep)
            {
                std::cerr << kTag << " the free arrived at step " << int(step) << ", expected "
                          << int(kInteropFreeStep) << std::endl;
                failed = true;
                break;
            }
            // Python freed the pool before sending this, but the daemon's
            // notification is a separate message; wait for it rather than
            // assume it has landed.
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (freed.find(pool_id) == freed.end() &&
                   std::chrono::steady_clock::now() < deadline)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                drain_freed();
            }
            if (freed.find(pool_id) == freed.end())
            {
                std::cerr << kTag << " no free notification for `" << pool_id << "` arrived"
                          << std::endl;
                failed = true;
                break;
            }
#ifdef __CUDACC__
            // The drain is the only place that could have done this, and it had
            // to before the view could be dropped or the segment unlinked.
            if (frames->payload() != nullptr)
            {
                std::cerr << kTag << " the mapping of `" << pool_id
                          << "` is still registered after the free notification" << std::endl;
                failed = true;
                break;
            }
#endif
            if (view_is_alive(frames->view()))
            {
                std::cerr << kTag << " the view of `" << pool_id << "` is still alive after the "
                             "free"
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

            bool threw = false;
            try
            {
                dora::PoolReadGuard read(frames->view());
                (void)read.size();
            }
            catch (const std::exception &)
            {
                threw = true;
            }
            if (!threw)
            {
                std::cerr << kTag << " a read guard opened on the freed pool" << std::endl;
                failed = true;
                break;
            }
            std::cout << kTag << " Python's pool is permanently unreadable after its free"
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

        // Every step is acked, hello and free included. The sender advances on
        // the ack of the step it is waiting for, so a kind this node handled but
        // did not ack stalls the whole script.
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

#ifdef __CUDACC__
    cudaFree(device_verdict);
    std::cout << kTag << " " << verified
              << " Python-written frames verified through the device alias, " << unavailable
              << " unavailable" << std::endl;
#else
    std::cout << kTag << " " << verified << " Python-written frames verified (" << copied
              << " copied, " << zero_copy << " zero-copy), " << unavailable << " unavailable"
              << std::endl;
#endif

    // The second release point, for the CUDA build: the destructor unregisters
    // and only then drops the view.
    frames.reset();

    if (failed || !done || verified != kInteropFrameSteps || unavailable != 1)
    {
        std::cerr << kTag << " the run did not complete the script" << std::endl;
        return 1;
    }
#ifndef __CUDACC__
    if (copied == 0 || zero_copy == 0)
    {
        std::cerr << kTag << " one of the two read paths was never exercised" << std::endl;
        return 1;
    }
#endif
    return 0;
}
