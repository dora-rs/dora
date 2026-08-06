//! Segment naming.
//!
//! The daemon only unlinks (and only orphan-sweeps) segments whose name starts
//! with `dora_pool_` and contains neither `/` nor `..` — see
//! `MemoryPoolManager::free_shared_memory`. Building the name in one validated
//! place is what keeps a mistyped pool id from leaking a segment in `/dev/shm`
//! for the life of the machine.
//!
//! Each component (`dataflow_id`, `node_id`, `pool_id`) is validated against
//! `NodeId`'s charset (`libraries/message/src/id.rs`), plus an explicit `..`
//! rejection: `NodeId` bans only a *leading* dot, so `a..b` is a valid
//! `NodeId` and remains reachable once `.` is allowed here. Without the
//! explicit check, the daemon's own `contains("..")` guard would no longer be
//! satisfiable by construction.

pub const SEGMENT_PREFIX: &str = "dora_pool_";

/// Longest name `shm_open` actually accepts on Linux glibc — measured
/// directly with a `shm_open` probe (glibc 2.31, aarch64): 253 bytes
/// succeeds, 254 and 255 both fail with `EINVAL`. glibc rejects a POSIX shm
/// name once `strlen(name) + 1 >= NAME_MAX` (`NAME_MAX` == 255), so the
/// longest name that actually works is 253, not 255. There is no leading
/// slash to subtract here: glibc strips any leading slash and prepends
/// `/dev/shm/` internally, and the name we build never has one to begin
/// with.
const MAX_SEGMENT_NAME_LEN: usize = 253;

fn validate_component(label: &str, value: &str) -> Result<(), String> {
    if value.is_empty() {
        return Err(format!("{label} must not be empty"));
    }
    // Checked before the charset test: once `.` is allowed, `..` is no
    // longer unreachable by construction, and it must produce this message,
    // not the charset one, so callers can tell traversal apart from a
    // stray character.
    if value.contains("..") {
        return Err(format!(
            "{label} `{value}` must not contain `..` (path traversal)"
        ));
    }
    if !value
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-' || c == '.')
    {
        return Err(format!(
            "{label} `{value}` may only contain ASCII letters, digits, `_`, `-` and `.`"
        ));
    }
    Ok(())
}

/// Build the `/dev/shm` segment name for a pool.
///
/// `pool_id` is the id the node chose and the same string the daemon's table is
/// keyed by, so it must be unique within the dataflow.
///
/// `cleanup_orphans` sweeps by matching the prefix `dora_pool_{dataflow_id}_`;
/// that match is unambiguous only because `DataflowId` is a UUID and so never
/// contains `_`. The charset here permits `_` in `dataflow_id`, so a
/// non-UUID dataflow id could in principle make one dataflow's sweep match a
/// segment belonging to another. Not reachable today — `DataflowId` is
/// always a UUID — but worth knowing if that ever changes.
pub fn segment_name(dataflow_id: &str, node_id: &str, pool_id: &str) -> Result<String, String> {
    validate_component("dataflow id", dataflow_id)?;
    validate_component("node id", node_id)?;
    validate_component("pool id", pool_id)?;

    let name = format!("{SEGMENT_PREFIX}{dataflow_id}_{node_id}_{pool_id}");
    if name.len() > MAX_SEGMENT_NAME_LEN {
        return Err(format!(
            "segment name `{name}` is too long: {} bytes, limit is {MAX_SEGMENT_NAME_LEN}",
            name.len()
        ));
    }
    Ok(name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn builds_a_name_the_daemon_will_unlink() {
        let name = segment_name("df-uuid", "eo_media", "eo_frames").expect("valid");
        assert_eq!(name, "dora_pool_df-uuid_eo_media_eo_frames");
        // Exactly the checks in MemoryPoolManager::free_shared_memory.
        assert!(name.starts_with("dora_pool_"));
        assert!(!name.contains('/'));
        assert!(!name.contains(".."));
    }

    #[test]
    fn rejects_path_traversal_in_any_component() {
        assert!(segment_name("df", "node", "../etc/passwd").is_err());
        assert!(segment_name("df", "../node", "pool").is_err());
        assert!(segment_name("../df", "node", "pool").is_err());
        assert!(segment_name("df", "node", "a/b").is_err());
    }

    #[test]
    fn rejects_a_double_dot_component_with_the_traversal_message_not_the_charset_message() {
        let err = segment_name("df", "node", "a..b").unwrap_err();
        assert!(err.contains("traversal"), "unexpected error: {err}");
        assert!(
            !err.contains("may only contain"),
            "charset message leaked: {err}"
        );
    }

    #[test]
    fn rejects_an_empty_pool_id() {
        let err = segment_name("df", "node", "").unwrap_err();
        assert!(err.contains("empty"), "unexpected error: {err}");
    }

    /// Linux caps a usable POSIX shm name at 253 bytes (measured; see
    /// `MAX_SEGMENT_NAME_LEN`). Failing here with a clear message beats a
    /// bare EINVAL from shm_open.
    #[test]
    fn rejects_a_name_longer_than_the_measured_limit() {
        let long = "x".repeat(240);
        let err = segment_name("df", "node", &long).unwrap_err();
        assert!(err.contains("too long"), "unexpected error: {err}");
    }

    /// Pins the exact boundary shm_open enforces, computed from the constant
    /// rather than hardcoded, so an off-by-one in the comparison operator
    /// cannot hide behind a long-and-obviously-too-long test case.
    #[test]
    fn allows_a_name_at_the_measured_limit_and_rejects_one_byte_over() {
        let fixed_len = SEGMENT_PREFIX.len() + "df".len() + 1 + "node".len() + 1;
        let pool_id_len = MAX_SEGMENT_NAME_LEN - fixed_len;

        let at_limit = "x".repeat(pool_id_len);
        let name = segment_name("df", "node", &at_limit).expect("exactly at the limit");
        assert_eq!(name.len(), MAX_SEGMENT_NAME_LEN);

        let over_limit = "x".repeat(pool_id_len + 1);
        assert!(segment_name("df", "node", &over_limit).is_err());
    }

    #[test]
    fn rejects_characters_outside_the_allowed_set() {
        assert!(segment_name("df", "node", "pool id").is_err());
        assert!(segment_name("df", "node", "pool\0id").is_err());
        assert!(segment_name("df", "node", "pool$id").is_err());
    }

    #[test]
    fn allows_the_characters_dora_node_and_dataflow_ids_actually_use() {
        assert!(segment_name("0193-abcd-ef01", "eo-media_2", "frames_0").is_ok());
    }

    /// `NodeId` (libraries/message/src/id.rs) permits dots everywhere except
    /// a leading position, e.g. `camera.left`. Our charset must accept them
    /// too, or a dataflow that parses and spawns fine dies later at pool
    /// registration with an error the user cannot act on.
    #[test]
    fn allows_a_dot_containing_node_id_like_node_id_validation_does() {
        assert!(segment_name("df", "camera.left", "pool").is_ok());
    }
}
