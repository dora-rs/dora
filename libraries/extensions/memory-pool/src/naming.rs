//! Segment naming.
//!
//! The daemon only unlinks (and only orphan-sweeps) segments whose name starts
//! with `dora_pool_` and contains neither `/` nor `..` — see
//! `MemoryPoolManager::free_shared_memory`. Building the name in one validated
//! place is what keeps a mistyped pool id from leaking a segment in `/dev/shm`
//! for the life of the machine.

pub const SEGMENT_PREFIX: &str = "dora_pool_";

/// Longest POSIX shared-memory name Linux accepts (`NAME_MAX`), minus the
/// leading slash the kernel adds.
const NAME_MAX: usize = 255;

fn validate_component(label: &str, value: &str) -> Result<(), String> {
    if value.is_empty() {
        return Err(format!("{label} must not be empty"));
    }
    if !value
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    {
        return Err(format!(
            "{label} `{value}` may only contain ASCII letters, digits, `_` and `-`"
        ));
    }
    Ok(())
}

/// Build the `/dev/shm` segment name for a pool.
///
/// `pool_id` is the id the node chose and the same string the daemon's table is
/// keyed by, so it must be unique within the dataflow.
pub fn segment_name(dataflow_id: &str, node_id: &str, pool_id: &str) -> Result<String, String> {
    validate_component("dataflow id", dataflow_id)?;
    validate_component("node id", node_id)?;
    validate_component("pool id", pool_id)?;

    let name = format!("{SEGMENT_PREFIX}{dataflow_id}_{node_id}_{pool_id}");
    if name.len() > NAME_MAX {
        return Err(format!(
            "segment name `{name}` is too long: {} bytes, limit is {NAME_MAX}",
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
    fn rejects_an_empty_pool_id() {
        let err = segment_name("df", "node", "").unwrap_err();
        assert!(err.contains("empty"), "unexpected error: {err}");
    }

    /// Linux caps a POSIX shm name at NAME_MAX (255). Failing here with a clear
    /// message beats a bare ENAMETOOLONG from shm_open.
    #[test]
    fn rejects_a_name_longer_than_name_max() {
        let long = "x".repeat(240);
        let err = segment_name("df", "node", &long).unwrap_err();
        assert!(err.contains("too long"), "unexpected error: {err}");
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
}
