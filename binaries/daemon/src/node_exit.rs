//! Classifying node exit statuses and the timeouts that decide when a
//! node should be killed.

use dora_message::common::NodeExitStatus;
use std::{sync::atomic, time::Duration};

/// Grace period used when the finish-straggler watchdog is enabled but
/// `DORA_FINISH_DRAIN_GRACE_SECS` is set to an unparseable value. Conservative:
/// a sink may legitimately keep working for a while after its inputs close
/// (flushing recordings, final writes).
pub(crate) const DEFAULT_FINISH_DRAIN_GRACE: Duration = Duration::from_secs(120);

/// Windows `STATUS_CONTROL_C_EXIT`. A process terminated by an unhandled
/// console `CTRL_C` / `CTRL_BREAK` event exits with this NTSTATUS, which Rust's
/// [`std::process::ExitStatus::code`] surfaces as this `i32` (`0xC000013A`).
/// The daemon's Windows `SoftKill` stops nodes with
/// `GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT)` (see `running_dataflow.rs`), so
/// a node that doesn't install its own console handler reports this code on a
/// planned stop — the Windows analog of Unix `Signal(15)` or the `143` wrapper
/// exit (dora-rs/dora#2425).
pub(crate) const STATUS_CONTROL_C_EXIT: i32 = -1073741510;

/// Whether `exit_status` has the *shape* of a node that exited because the
/// daemon asked it to stop (SoftKill), rather than because of an application
/// error.
///
/// Callers must additionally gate on `grace_duration_kills` — this predicate
/// only recognises the shape of a stop-induced exit, not whether the daemon
/// actually initiated one. A node that produces one of these codes on its own
/// (without a preceding daemon SoftKill) is still reported as a failure.
///
/// - Unix: `SIGTERM` (15) / `SIGINT` (2) surface as `Signal`.
/// - Wrappers such as `uv run python` catch the signal and exit `128 + signo`
///   (143 / 130) instead of propagating it, so the code appears as `ExitCode`.
/// - Windows: an unhandled `CTRL_BREAK_EVENT` terminates the node with
///   [`STATUS_CONTROL_C_EXIT`] (dora-rs/dora#2425).
pub(crate) fn is_sigterm_like_exit(exit_status: &NodeExitStatus) -> bool {
    matches!(
        exit_status,
        NodeExitStatus::Signal(15)
            | NodeExitStatus::Signal(2)
            | NodeExitStatus::ExitCode(143)
            | NodeExitStatus::ExitCode(130)
            | NodeExitStatus::ExitCode(STATUS_CONTROL_C_EXIT)
    )
}

/// Whether `exit_status` has the shape of a node terminated by SIGKILL (or process kill).
///
/// On Unix, a process killed by `ProcessOperation::Kill` (`start_kill()`) terminates
/// with `Signal(9)` (SIGKILL) or exit code 137 (= 128 + 9).
/// On non-Unix platforms, a killed process terminates with a non-zero status.
pub(crate) fn is_sigkill_like_exit(exit_status: &NodeExitStatus) -> bool {
    #[cfg(unix)]
    {
        matches!(
            exit_status,
            NodeExitStatus::Signal(9) | NodeExitStatus::ExitCode(137)
        )
    }
    #[cfg(not(unix))]
    {
        !exit_status.is_success()
    }
}

/// Decide whether the health-check watchdog should kill a node.
/// Pure predicate for startup watchdog decision (#3022).
///
/// Returns `true` if a spawned process is unconnected and the elapsed time since
/// process spawn exceeds `timeout`.
///
/// Returns `false` if `connected` is true or `spawned_at_millis` is 0 (process has
/// not yet spawned or is sleeping in restart backoff).
pub(crate) fn startup_timeout_should_kill(
    connected: bool,
    spawned_at_millis: u64,
    now_millis: u64,
    timeout: Duration,
) -> bool {
    if connected || spawned_at_millis == 0 {
        return false;
    }
    let elapsed_ms = now_millis.saturating_sub(spawned_at_millis);
    u128::from(elapsed_ms) > timeout.as_millis()
}

/// Returns `true` only once the node has **connected** at least once (sent its
/// first `DaemonRequest`) and has then been silent for longer than `timeout`.
///
/// A node that has not yet connected is still starting up and must never be
/// killed by this watchdog: `last_activity` is seeded to the spawn timestamp
/// (see `spawner.rs` / `prepared.rs`), so without the `connected` gate the
/// timeout clock would start ticking at spawn — before the node has had any
/// chance to communicate — and a node whose legitimate cold start exceeds
/// `health_check_timeout` would be SIGKILLed mid-startup, mirroring the trap
/// the finish-straggler watchdog was explicitly fixed for (#2937).
pub(crate) fn health_check_should_kill(
    connected: bool,
    last_activity_millis: u64,
    now_millis: u64,
    timeout: Duration,
) -> bool {
    if !connected {
        return false;
    }
    let elapsed_ms = now_millis.saturating_sub(last_activity_millis);
    u128::from(elapsed_ms) > timeout.as_millis()
}

/// Grace period before the finish-straggler watchdog escalates a stuck node, or
/// `None` if the watchdog has been explicitly **disabled**.
///
/// The watchdog is **on by default** (dora-rs/dora#2270): a node still stuck
/// past the grace once its dataflow is otherwise finished is escalated rather
/// than hanging until an external timeout. `DORA_FINISH_DRAIN_GRACE_SECS` tunes
/// it — a whole number of seconds sets the grace; `off`/`disabled` opts out
/// entirely (the escape hatch for a deployment that hits a false positive). It
/// shipped dark first and was validated on nightly + a deterministic e2e
/// (#2271, #2276, #2280) before this default flip.
pub(crate) fn finish_drain_grace() -> Option<Duration> {
    parse_finish_drain_grace(
        std::env::var("DORA_FINISH_DRAIN_GRACE_SECS")
            .ok()
            .as_deref(),
    )
}

pub(crate) fn parse_finish_drain_grace(value: Option<&str>) -> Option<Duration> {
    let Some(value) = value else {
        // unset → enabled at the conservative default grace (on by default)
        return Some(DEFAULT_FINISH_DRAIN_GRACE);
    };
    // explicit opt-out escape hatch for a deployment that hits a false positive
    if value.eq_ignore_ascii_case("off") || value.eq_ignore_ascii_case("disabled") {
        return None;
    }
    match value.parse::<u64>() {
        Ok(secs) => Some(Duration::from_secs(secs)),
        Err(_) => {
            static WARNED: std::sync::atomic::AtomicBool =
                std::sync::atomic::AtomicBool::new(false);
            if !WARNED.swap(true, atomic::Ordering::Relaxed) {
                tracing::warn!(
                    "invalid DORA_FINISH_DRAIN_GRACE_SECS value `{value}` \
                     (expected whole seconds or `off`); using the default of {}s",
                    DEFAULT_FINISH_DRAIN_GRACE.as_secs()
                );
            }
            Some(DEFAULT_FINISH_DRAIN_GRACE)
        }
    }
}

#[cfg(test)]
mod planned_stop_exit_tests {
    use super::{STATUS_CONTROL_C_EXIT, is_sigkill_like_exit, is_sigterm_like_exit};
    use dora_message::common::NodeExitStatus;

    #[test]
    fn status_control_c_exit_constant_matches_ntstatus() {
        // STATUS_CONTROL_C_EXIT = 0xC000013A, surfaced by
        // `ExitStatus::code()` (u32 -> i32) on Windows.
        assert_eq!(STATUS_CONTROL_C_EXIT, 0xC000013Au32 as i32);
        assert_eq!(STATUS_CONTROL_C_EXIT, -1073741510);
    }

    #[test]
    fn recognises_planned_stop_exit_shapes() {
        // Unix SIGTERM / SIGINT.
        assert!(is_sigterm_like_exit(&NodeExitStatus::Signal(15)));
        assert!(is_sigterm_like_exit(&NodeExitStatus::Signal(2)));
        // Wrapper (`uv run python`) that catches the signal and exits 128 + signo.
        assert!(is_sigterm_like_exit(&NodeExitStatus::ExitCode(143)));
        assert!(is_sigterm_like_exit(&NodeExitStatus::ExitCode(130)));
        // Windows: unhandled CTRL_BREAK_EVENT -> STATUS_CONTROL_C_EXIT (dora-rs/dora#2425).
        assert!(is_sigterm_like_exit(&NodeExitStatus::ExitCode(
            STATUS_CONTROL_C_EXIT
        )));
    }

    #[test]
    fn does_not_recognise_genuine_failures() {
        assert!(!is_sigterm_like_exit(&NodeExitStatus::Success));
        assert!(!is_sigterm_like_exit(&NodeExitStatus::ExitCode(1)));
        assert!(!is_sigterm_like_exit(&NodeExitStatus::ExitCode(-1)));
        assert!(!is_sigterm_like_exit(&NodeExitStatus::Unknown));
        // SIGKILL is a hard kill (grace exceeded), not a graceful stop —
        // it must keep flowing through the GraceDuration branch.
        assert!(!is_sigterm_like_exit(&NodeExitStatus::Signal(9)));
    }

    #[test]
    fn recognises_kill_exit_shapes() {
        #[cfg(unix)]
        {
            assert!(is_sigkill_like_exit(&NodeExitStatus::Signal(9)));
            assert!(is_sigkill_like_exit(&NodeExitStatus::ExitCode(137)));
            assert!(!is_sigkill_like_exit(&NodeExitStatus::ExitCode(1)));
            assert!(!is_sigkill_like_exit(&NodeExitStatus::Signal(15)));
        }
        #[cfg(not(unix))]
        {
            assert!(is_sigkill_like_exit(&NodeExitStatus::ExitCode(1)));
            assert!(!is_sigkill_like_exit(&NodeExitStatus::Success));
        }
    }
}

#[cfg(test)]
mod health_check_tests {
    use super::health_check_should_kill;
    use std::time::Duration;

    const TIMEOUT: Duration = Duration::from_secs(5);

    #[test]
    fn not_connected_is_never_killed_even_when_long_silent() {
        // Regression for #2937: `last_activity` is seeded to the spawn
        // timestamp, so a node in a slow cold start (imports + model-weight
        // load) reads as long-silent well before it ever connects. The
        // watchdog must not kill it while it is still starting up — even if
        // the elapsed time since spawn already exceeds the timeout.
        let spawn = 1_000u64;
        let now = spawn + 10_000; // 10s after spawn, timeout is 5s
        assert!(!health_check_should_kill(false, spawn, now, TIMEOUT));
    }

    #[test]
    fn connected_and_silent_past_timeout_is_killed() {
        let last = 1_000u64;
        let now = last + 6_000; // 6s of post-connection silence, timeout 5s
        assert!(health_check_should_kill(true, last, now, TIMEOUT));
    }

    #[test]
    fn connected_and_recently_active_is_not_killed() {
        let last = 1_000u64;
        let now = last + 4_000; // 4s < 5s timeout
        assert!(!health_check_should_kill(true, last, now, TIMEOUT));
    }

    #[test]
    fn exactly_at_timeout_is_not_killed() {
        // The comparison is strictly greater-than, so a node silent for
        // exactly the timeout is given the benefit of the doubt.
        let last = 1_000u64;
        let now = last + 5_000;
        assert!(!health_check_should_kill(true, last, now, TIMEOUT));
    }

    #[test]
    fn clock_skew_does_not_underflow() {
        // `now` before `last` (clock went backwards) must not panic via
        // subtraction underflow, and must not be treated as elapsed time.
        let last = 10_000u64;
        let now = 1_000u64;
        assert!(!health_check_should_kill(true, last, now, TIMEOUT));
    }
}

#[cfg(test)]
mod startup_timeout_tests {
    use super::startup_timeout_should_kill;
    use std::time::Duration;

    const TIMEOUT: Duration = Duration::from_secs(5);

    #[test]
    fn unspawned_or_in_backoff_is_never_killed() {
        // spawned_at_millis == 0 represents unspawned or in restart backoff sleep
        let now = 10_000u64;
        assert!(!startup_timeout_should_kill(false, 0, now, TIMEOUT));
    }

    #[test]
    fn connected_node_is_never_killed_by_startup_watchdog() {
        // Connected nodes are governed by health_check_timeout, not startup_timeout
        let spawn = 1_000u64;
        let now = spawn + 10_000;
        assert!(!startup_timeout_should_kill(true, spawn, now, TIMEOUT));
    }

    #[test]
    fn unconnected_node_within_timeout_is_not_killed() {
        let spawn = 1_000u64;
        let now = spawn + 4_000; // 4s < 5s
        assert!(!startup_timeout_should_kill(false, spawn, now, TIMEOUT));
    }

    #[test]
    fn unconnected_node_past_timeout_is_killed() {
        let spawn = 1_000u64;
        let now = spawn + 6_000; // 6s > 5s
        assert!(startup_timeout_should_kill(false, spawn, now, TIMEOUT));
    }

    #[test]
    fn exactly_at_timeout_is_not_killed() {
        let spawn = 1_000u64;
        let now = spawn + 5_000;
        assert!(!startup_timeout_should_kill(false, spawn, now, TIMEOUT));
    }

    #[test]
    fn clock_skew_does_not_underflow() {
        let spawn = 10_000u64;
        let now = 1_000u64;
        assert!(!startup_timeout_should_kill(false, spawn, now, TIMEOUT));
    }
}
