// Assertions both consumers make, in the one place they can be shared.
//
// The CPU consumer (`pool-receiver.cc`) and the CUDA one
// (`pool-receiver-cuda.cc`) read the same pools from the same producer, so the
// checks that are about the *pool* rather than about how its bytes are reached
// belong to both. Everything specific to one of them — the CPU ring scan, the
// kernel launch — stays in its own file.
//
// Header-only and CUDA-free: `pool-receiver-cuda.cc` is compiled by nvcc, so
// nothing here may need a compiler feature past C++17.

#pragma once

#include <dora/memory_pool.hpp>

#include <algorithm>
#include <iostream>
#include <string>

#include "protocol.h"

namespace pool_example
{

inline const char *outcome_name(dora::PoolReadOutcome outcome)
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

/// The segment name can only have come from the daemon.
///
/// A consumer passes `read_memory_pool` a pool id and nothing else. The name it
/// gets back is `dora_pool_<dataflow-uuid>_<owning-node-id>_<pool-id>`: the
/// dataflow uuid is minted per run and the owning node id belongs to the other
/// node, so neither is anything the consumer could have derived from the id it
/// asked with.
///
/// **This hard-codes the segment grammar on purpose** — it is a contract lock
/// on `libraries/extensions/memory-pool/src/naming.rs::segment_name`, which is
/// what makes "the consumer did not guess the name" checkable at all. A
/// deliberate change to that grammar (or to `DataflowId` no longer being a
/// uuid) will fail here, and this is the reason why.
///
/// `owner_tail` is everything the name carries after the uuid. It is a
/// parameter because the two producers spell it differently, and *that
/// difference is itself part of what the check pins*:
///
///   * the C++ producer builds both the pool id and the segment name from
///     `naming::segment_name`, so the tail is `<node-id>_<pool-id>`;
///   * the Python binding formats its segment name (`dora_pool_{dataflow}_
///     {node}_{counter}`) and its buffer id (`pool_{node}_{counter}`) in two
///     separate `format!` calls that share no code, so the tail is
///     `<node-id>_<counter>` and the pool id is *not* a substring of the name.
///
/// A single `pool_id`-derived suffix could only serve the first.
inline bool segment_name_from_daemon(const std::string &name, const std::string &owner_tail,
                                     const std::string &pool_id, const char *tag)
{
    const std::string prefix = "dora_pool_";
    const std::string suffix = "_" + owner_tail;
    if (name.size() <= prefix.size() + suffix.size() ||
        name.compare(0, prefix.size(), prefix) != 0 ||
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0)
    {
        std::cerr << tag << " segment name `" << name << "` is not the daemon's name for pool `"
                  << pool_id << "`" << std::endl;
        return false;
    }
    const std::string dataflow_id =
        name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
    if (dataflow_id.size() != 36 || std::count(dataflow_id.begin(), dataflow_id.end(), '-') != 4)
    {
        std::cerr << tag << " segment name `" << name << "` carries `" << dataflow_id
                  << "` where the dataflow uuid should be" << std::endl;
        return false;
    }
    return true;
}

/// The C++ producer's spelling: it derives the pool id and the segment name
/// from the same `naming::segment_name`, so the id is the name's last
/// component.
inline bool segment_name_from_daemon(const std::string &name, const std::string &pool_id,
                                     const char *tag)
{
    return segment_name_from_daemon(name, std::string(kSenderNodeId) + "_" + pool_id, pool_id, tag);
}

} // namespace pool_example
