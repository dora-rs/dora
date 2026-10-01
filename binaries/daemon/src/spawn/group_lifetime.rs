//! What happens to a node's process group after the node process itself is gone.
//!
//! Split out from the spawn path on purpose (#3472 review): this is the logic
//! that runs for every unix node, whether it was started by `dora run` or
//! attached by `dora up`, and it is a different question from the one
//! `prepared.rs` answers about a node that is still alive — the in-node guard
//! and the shell guard live there. Keeping the two apart is what makes either
//! reviewable. The one exception is a group a node *abandoned*: that is only
//! taken on the `dora run` path, which is the whole of #3472's ask and no more
//! (see [`contain_exited_group`]).

/// The two instants a [`crate::ProcessOperation::StopRequested`] marks, so that
/// a group whose leader has already exited can be given the same treatment it
/// would have had if the leader were still there to receive it.
pub(super) type StopLadder = (tokio::time::Instant, tokio::time::Instant);

/// Contain what a node left behind in its process group, the moment the node
/// process itself is reaped.
///
/// Every node is spawned as its own group leader (`ProcessGroup::leader()`), so
/// `pgid == pid`. A fire-and-forget background fork (`sh -c 'cmd &'`) outlives the
/// node process and is then unreachable by the stop ladder's group kill, so the
/// group is killed here. The in-node/shell-guard containment only runs while the
/// node process is alive, which makes this the one place that catches a fork
/// abandoned by a node that finished normally (dora-rs/dora#3472).
///
/// # What a group a node abandoned is worth
///
/// `bind_to_run_parent` is the caller saying the node was spawned on the
/// in-process `dora run` / `Daemon::run_dataflow` path (`bind_nodes_to_parent`,
/// which is what injects `DORA_RUN_PARENT_PID`), as opposed to the
/// coordinator-attached `dora up` path. It decides one case only, the one with
/// no stop to wait for: a node that exited on its own.
///
/// On `dora run` such a group is killed, and that is #3472. The node is what
/// owned that group, and it goes away with the process-wait task that was
/// watching it, so nothing is left that can reach the fork — not the node, since
/// it has exited, and not the daemon, since its task for that node has ended.
/// Whatever the fork holds stays held until `dora run` itself exits, which for
/// an inherited stdout is the hang #3472 reports, not a feature.
///
/// Note that this fires whenever a node exits on its own, not only as the run
/// winds down: a source that sent its N messages and returned leaves the same
/// unreachable group whether the dataflow has three seconds left or three
/// minutes. The orphan is the same either way, and it is the orphan, not the
/// countdown, that the SIGKILL is for.
///
/// On `dora up` the group is left alone, deliberately. That dataflow outlives
/// the node: a node that starts a viewer, a helper server or a launcher is
/// allowed to exit without them, and a child the node itself stopped just before
/// returning — `proc.terminate()`, a file still being flushed — is mid-cleanup,
/// not abandoned. `SIGKILL`ing those with no grace period, no log line and no way
/// to opt out would be a silent 1.0 behaviour change well beyond this PR's
/// title, so it takes the narrower reading instead. A dataflow that wants its
/// strays reaped says so by stopping the node, which is the case below.
///
/// `stop` is the exception, set when a stop is in flight, and it covers two
/// cases: a node the ladder already signalled, and — the subtler one — a node
/// that honored the `NodeEvent::Stop` and exited during the grace period, before
/// any signal was due. In both the group was asked to shut down and its members
/// may be mid-cleanup, so killing it now would cut the stop grace period short
/// for every one of them (#3472 review). The escalation is the ladder's to make,
/// then, and this replays it onto the group: `SIGTERM` at the soft-kill instant,
/// `SIGKILL` at the escalation deadline. That half is not gated on the spawn
/// path: a group that was asked to stop is expected to end, on every path.
///
/// Replaying it matters because a node that stops *promptly* would otherwise be
/// the worst case for its children: the ladder's own `SIGTERM` was going to the
/// node process, which is already gone, so a prompt node's children would be
/// `SIGKILL`ed having never been asked to stop (#3472 review) — a node returning
/// from `STOP` without terminating the subprocess it started loses its file
/// mid-write instead of getting the signal it would have got had it hung on.
///
/// `signalled` is the caller saying it already ran a `SoftKill` or `Kill`, which
/// `process-wrap` turns into a `killpg` on the whole group — so replaying the
/// `SIGTERM` here would deliver a second one, to a group that has just been told
/// to stop, killing whatever a child's `SIGTERM` handler started on the way out
/// (#3472 review).
///
/// This runs before the node's exit is reported (see `prepared.rs`'s
/// `finished_tx` send), so a dataflow cannot finish — and `dora run` cannot
/// exit, cancelling the wait — while a group it deferred is still outstanding.
#[cfg(unix)]
pub(super) async fn contain_exited_group(
    pid: u32,
    stop: Option<StopLadder>,
    signalled: bool,
    bind_to_run_parent: bool,
) {
    /// How often the group is re-checked while waiting: short enough that the
    /// wait ends right after the last member exits, long enough to be free.
    const POLL: std::time::Duration = std::time::Duration::from_millis(250);

    let Some((soft_kill_at, kill_at)) = stop else {
        // Nothing is waiting for this group: its members are abandoned, which
        // only the `dora run` path acts on. Everywhere else they are somebody's
        // to look after, and the node never said to stop them.
        if !bind_to_run_parent {
            return;
        }
        // A node that did its work and exited leaves an empty group behind, and
        // that is the common case by far: a source that sends N messages and
        // returns, anything that stops once its inputs close. Nothing was
        // abandoned, so there is nothing to signal and nothing to say — and
        // `dora run` prints warnings, so a warning here would tell every user
        // with a short-lived node that their node had been SIGKILLed. The
        // kernel's answer is the whole one, for the same reason as in the loop
        // below: the leader is reaped, so a group still holding a member holds
        // something alive.
        if !group_has_members(pid) {
            return;
        }
        tracing::warn!(
            "node process group {pid} was abandoned by a node that exited on its own; \
             SIGKILLing whatever is left in it (dora run)"
        );
        signal_group(pid, libc::SIGKILL);
        return;
    };
    let mut replayed = signalled;

    loop {
        // Re-checking is also what keeps the wait from outliving the group: the
        // leader is reaped, so its pgid can be recycled once the last member is
        // gone, and a recycled group is not ours to kill. Bailing out as soon as
        // the group is empty keeps that window to a poll interval instead of the
        // rest of the grace period.
        //
        // And the kernel's answer is the whole one here: the leader is reaped
        // before this runs, so a group still holding a member holds something
        // alive, not just a corpse. `killpg(pgid, 0)` does count a zombie, so
        // a reparented grandchild that has exited but that its new parent has
        // not reaped yet reads as a member. That only costs a no-op signal and,
        // on the `dora run` branch above, a warning about a group that is
        // already down — the alternative was reading the group as empty and
        // walking away from a live orphan (#3472 review).
        if !group_has_members(pid) {
            return;
        }
        let now = tokio::time::Instant::now();
        if now >= kill_at {
            tracing::warn!(
                "node process group {pid} ignored the stop grace period; \
                 SIGKILLing what is left of it"
            );
            signal_group(pid, libc::SIGKILL);
            return;
        }
        if !replayed && now >= soft_kill_at {
            signal_group(pid, libc::SIGTERM);
            replayed = true;
            continue;
        }
        tokio::time::sleep(POLL.min(kill_at.saturating_duration_since(now))).await;
    }
}

/// Whether the process group still has a member left in it.
#[cfg(unix)]
fn group_has_members(pid: u32) -> bool {
    // SAFETY: signal 0 performs error checking only and sends no signal.
    unsafe { libc::killpg(pid as libc::pid_t, 0) == 0 }
}

/// Signal the whole group, best effort: an already-empty group just yields ESRCH.
#[cfg(unix)]
fn signal_group(pid: u32, signal: libc::c_int) {
    // SAFETY: the group this process spawned as its leader, and a pid always
    // fits the `pid_t` the kernel hands it out as.
    unsafe {
        libc::killpg(pid as libc::pid_t, signal);
    }
}

// `#[cfg(test)]` on its own line, deliberately: `scripts/qa/unwrap-budget.sh`
// excludes test blocks by that exact spelling, and a combined
// `#[cfg(all(test, unix))]` is counted as production code — which is how this
// module's test-only `.expect`s would land in the unwrap budget.
#[cfg(test)]
#[cfg(unix)]
mod tests {
    use super::*;

    /// Spawn a group-leading `sh` whose background child outlives it, as a node's
    /// process group looks to [`contain_exited_group`]: `pgid == pid`, and members
    /// that survive the leader unless the group is killed. `child_script` is run
    /// by `sh` as that background child, and the leader itself lives for
    /// `leader_hold_secs` — a leader that outlives its child is what keeps the
    /// group non-empty, so this is what a test makes the group "empty" with.
    /// Returns the leader and the child's pid.
    fn spawn_group_with_child(
        dir: &std::path::Path,
        child_script: &str,
        leader_hold_secs: u32,
    ) -> (std::process::Child, u32) {
        use std::os::unix::process::CommandExt as _;

        let script = dir.join("child.sh");
        std::fs::write(&script, child_script).expect("failed to write the child script");
        let pid_file = dir.join("pid");
        let mut leader = std::process::Command::new("sh");
        let mut leader = leader
            .arg("-c")
            .arg(format!(
                "sh {} & echo $! > {}; exec sleep {leader_hold_secs}",
                script.display(),
                pid_file.display()
            ))
            .process_group(0)
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .expect("failed to spawn the group leader");

        let child_pid = loop {
            if let Ok(Ok(pid)) =
                std::fs::read_to_string(&pid_file).map(|contents| contents.trim().parse::<u32>())
            {
                break pid;
            }
            assert!(
                leader.try_wait().expect("try_wait failed").is_none(),
                "the group leader exited before reporting its child pid"
            );
            std::thread::sleep(std::time::Duration::from_millis(50));
        };

        (leader, child_pid)
    }

    /// A child that only stops when the group is killed: it ignores the stop it
    /// is first asked for, so only the escalation can end it.
    const TERM_IGNORING_CHILD: &str = "trap '' TERM; sleep 300";

    /// Whether `pid` is still running, as opposed to merely present.
    ///
    /// `kill(pid, 0)` cannot tell those apart: a zombie is still a group member
    /// until it is reaped, so it answers 0 long after the process has stopped.
    /// These children are grandchildren of the test binary — their parent is
    /// the group's leader, so when the containment kills the group they are
    /// reparented to PID 1, and only PID 1 can reap them. Whether that has
    /// happened yet is PID 1's business, not ours, so a test that waited on
    /// `kill` alone would be asserting on the host's init. Linux exposes the
    /// state, so ask it; elsewhere, fall back to `kill` and hope.
    fn process_alive(pid: u32) -> bool {
        #[cfg(target_os = "linux")]
        if let Ok(stat) = std::fs::read_to_string(format!("/proc/{pid}/stat")) {
            // The comm field is parenthesised and may itself contain spaces and
            // parens, so the state is the first field after the *last* `)`.
            if let Some(state) = stat
                .rsplit_once(')')
                .and_then(|(_, rest)| rest.split_whitespace().next())
            {
                return state != "Z";
            }
        }
        // SAFETY: signal 0 performs error checking only and sends no signal.
        unsafe { libc::kill(pid as libc::pid_t, 0) == 0 }
    }

    /// The helper the rest of these tests lean on: a child that has exited but
    /// not been reaped is gone as far as they are concerned. Without this the
    /// suite would pass or fail on whether the host's PID 1 got round to it.
    #[tokio::test]
    async fn a_zombie_does_not_count_as_alive() {
        let mut child = tokio::process::Command::new("sh")
            .arg("-c")
            .arg("exit 0")
            .spawn()
            .unwrap();
        let pid = child.id().unwrap();
        // No `wait()`: the child stays a zombie, and a zombie is exactly the
        // state `kill(pid, 0)` still answers 0 for.
        tokio::time::sleep(std::time::Duration::from_millis(200)).await;
        #[cfg(target_os = "linux")]
        {
            let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).unwrap();
            let state = stat
                .rsplit_once(')')
                .unwrap()
                .1
                .split_whitespace()
                .next()
                .unwrap();
            assert_eq!(state, "Z", "the child should be an unreaped zombie here");
            // SAFETY: signal 0 performs error checking only and sends no signal.
            assert_eq!(
                unsafe { libc::kill(pid as libc::pid_t, 0) },
                0,
                "kill still reports a zombie as present, which is the whole reason \
                 this helper reads the state instead"
            );
        }
        assert!(
            !process_alive(pid),
            "a reaped-or-not, exited process must not read as alive"
        );
        let _ = child.start_kill();
    }

    async fn wait_for_exit(pid: u32, what: &str) {
        for _ in 0..200 {
            if !process_alive(pid) {
                return;
            }
            tokio::time::sleep(std::time::Duration::from_millis(50)).await;
        }
        panic!("{what} {pid} was still alive after 10s");
    }

    /// A node that exited on its own left a fork behind, and nothing else will
    /// ever clean it up: the group is killed at once, with no stop to wait for.
    /// This is the `dora run` path, which is the one that takes abandoned groups
    /// (#3472).
    #[tokio::test]
    async fn exited_node_group_is_killed_when_no_stop_is_in_flight() {
        let dir = tempfile::tempdir().unwrap();
        let (mut leader, child) = spawn_group_with_child(dir.path(), TERM_IGNORING_CHILD, 300);
        let started = std::time::Instant::now();

        contain_exited_group(leader.id(), None, false, true).await;

        wait_for_exit(child, "the abandoned fork").await;
        assert!(
            started.elapsed() < std::time::Duration::from_secs(5),
            "an abandoned group must be killed without waiting for a deadline"
        );
        let _ = leader.kill();
    }

    /// …but only on the `dora run` path. Off it, a group the node abandoned is
    /// left running: on `dora up` the dataflow outlives the node, so a helper
    /// the node started is somebody's to look after, and a child the node itself
    /// stopped just before returning is mid-cleanup rather than abandoned. The
    /// containment of #3472 is the `dora run` hang; taking every unix node's
    /// strays with it would be a 1.0 behaviour change this PR does not need.
    #[tokio::test]
    async fn abandoned_group_is_left_alone_off_the_dora_run_path() {
        let dir = tempfile::tempdir().unwrap();
        let (mut leader, child) = spawn_group_with_child(dir.path(), TERM_IGNORING_CHILD, 300);

        contain_exited_group(leader.id(), None, false, false).await;

        // Nothing waits on this group, so there is no deadline for it to miss:
        // a short settle is the whole assertion.
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        assert!(
            process_alive(child),
            "a group abandoned off the dora run path must be left alone, not SIGKILLed"
        );
        let _ = leader.kill();
        let _ = unsafe { libc::kill(child as libc::pid_t, libc::SIGKILL) };
    }

    /// A node that did its work and exited leaves an *empty* group behind,
    /// which is not an abandonment: nothing to kill, and — `dora run` prints
    /// warnings — nothing to say either (#3472 review).
    #[tokio::test]
    async fn a_node_that_exited_cleanly_is_not_reported_as_abandoned() {
        use std::os::unix::process::CommandExt as _;

        let mut leader = std::process::Command::new("sh")
            .arg("-c")
            .arg("exit 0")
            .process_group(0)
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .expect("failed to spawn the group leader");
        let exited = leader.wait().expect("failed to reap the group leader");
        assert!(exited.success());
        let pid = leader.id();

        let capture = crate::tests::LevelCapture::default();
        let _guard = tracing::subscriber::set_default(capture.clone());
        contain_exited_group(pid, None, false, true).await;
        drop(_guard);

        let levels = capture.levels.lock().unwrap();
        assert!(
            !levels.contains(&tracing::Level::WARN),
            "an empty group is not an abandoned one, and `dora run` prints warnings: {levels:?}"
        );
    }

    /// The counterpart, and the reason the check is not just a log guard: the
    /// group really is abandoned, and it is `SIGKILL`ed for it (#3472).
    #[tokio::test]
    async fn an_abandoned_group_is_killed_on_the_dora_run_path() {
        let dir = tempfile::tempdir().unwrap();
        let (mut leader, child) = spawn_group_with_child(dir.path(), TERM_IGNORING_CHILD, 300);

        let capture = crate::tests::LevelCapture::default();
        let _guard = tracing::subscriber::set_default(capture.clone());
        contain_exited_group(leader.id(), None, false, true).await;
        drop(_guard);

        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
        assert!(
            !process_alive(child),
            "a group abandoned on the dora run path must be SIGKILLed (#3472)"
        );
        assert!(
            capture
                .levels
                .lock()
                .unwrap()
                .contains(&tracing::Level::WARN),
            "killing a group the user cannot account for has to say so"
        );
        let _ = leader.kill();
    }

    /// The stop path: the group was already asked to shut down, so its members
    /// may be mid-cleanup. Nothing touches it before the ladder's own soft-kill
    /// instant, and the escalation is what ends it (#3472 review).
    #[tokio::test]
    async fn stopped_node_group_survives_until_the_ladder_kills_it() {
        let dir = tempfile::tempdir().unwrap();
        let (mut leader, child) = spawn_group_with_child(dir.path(), TERM_IGNORING_CHILD, 300);
        let leader_pid = leader.id();
        // Reap the leader while the containment runs, as the node's own
        // process-wait task does.
        let reaper = tokio::task::spawn_blocking(move || leader.wait());

        let started = tokio::time::Instant::now();
        let containment = tokio::spawn(async move {
            let now = tokio::time::Instant::now();
            contain_exited_group(
                leader_pid,
                Some((
                    now + std::time::Duration::from_secs(2),
                    now + std::time::Duration::from_secs(4),
                )),
                false,
                true,
            )
            .await
        });

        // Well past the soft-kill instant and short of the escalation, the child
        // is untouched: a group that a node was asked to shut down may be
        // mid-cleanup, and killing it early cuts the grace period short.
        tokio::time::sleep(std::time::Duration::from_millis(3_000)).await;
        assert!(
            process_alive(child),
            "the child must survive until the ladder's escalation, not be killed at the soft-kill \
             instant"
        );
        containment.await.expect("containment task panicked");
        wait_for_exit(child, "the child left mid-shutdown").await;
        assert!(
            started.elapsed() >= std::time::Duration::from_millis(3_500),
            "the containment must not return before the escalation deadline (returned after {:?})",
            started.elapsed()
        );
        let _ = reaper.await;
    }

    /// A node that stops promptly must leave its children no worse off than one
    /// that has to be chased: the group still gets the ladder's `SIGTERM`, at the
    /// soft-kill instant, and a child that shuts down on it is then left alone.
    #[tokio::test]
    async fn stopped_node_group_is_asked_to_stop_before_it_is_killed() {
        let dir = tempfile::tempdir().unwrap();
        let marker = dir.path().join("term");
        let (mut leader, child) = spawn_group_with_child(
            dir.path(),
            &format!(
                "trap 'echo term > {}; exit 0' TERM\nsleep 300",
                marker.display()
            ),
            1,
        );

        // The node exits on its own, before any signal is due, leaving the child
        // with nothing asked of it: the shape that has to go through the ladder.
        let leader_pid = leader.id();
        let reaper = tokio::task::spawn_blocking(move || leader.wait());
        let started = tokio::time::Instant::now();
        let now = tokio::time::Instant::now();
        contain_exited_group(
            leader_pid,
            Some((
                now + std::time::Duration::from_secs(2),
                now + std::time::Duration::from_secs(120),
            )),
            false,
            true,
        )
        .await;

        assert!(
            started.elapsed() < std::time::Duration::from_secs(20),
            "the child handled the SIGTERM, so nothing is left to kill (took {:?})",
            started.elapsed()
        );
        assert_eq!(
            std::fs::read_to_string(&marker).ok().as_deref(),
            Some("term\n"),
            "the group must get the ladder's SIGTERM, not be left to the SIGKILL alone"
        );
        wait_for_exit(child, "the child that handles SIGTERM").await;
        let _ = reaper.await;
    }

    /// …but the group is never asked to stop twice. A node that ignored `Stop`
    /// and then died from the ladder's `SIGTERM` has already had the whole group
    /// signalled by that signal, since `process-wrap` sends it with `killpg`;
    /// replaying it here delivers a second one, killing whatever a child's
    /// handler started on the way out (#3472 review).
    #[tokio::test]
    async fn a_group_the_ladder_already_signalled_is_not_asked_twice() {
        /// A child that records every `SIGTERM` it gets, then goes.
        fn term_counting_child(count_file: &std::path::Path) -> String {
            format!(
                "trap 'echo x >> {}; exit 0' TERM\nsleep 300",
                count_file.display()
            )
        }

        let dir = tempfile::tempdir().unwrap();
        let ladder_signalled = dir.path().join("signalled");
        let (mut leader, child) =
            spawn_group_with_child(dir.path(), &term_counting_child(&ladder_signalled), 1);
        let leader_pid = leader.id();
        let reaper = tokio::task::spawn_blocking(move || leader.wait());
        let now = tokio::time::Instant::now();
        contain_exited_group(
            leader_pid,
            Some((
                now + std::time::Duration::from_secs(1),
                now + std::time::Duration::from_secs(3),
            )),
            true,
            true,
        )
        .await;
        assert!(
            !ladder_signalled.exists(),
            "the group was already sent the ladder's SIGTERM, so it must not be sent again"
        );
        wait_for_exit(child, "the child left to the escalation").await;
        let _ = reaper.await;

        // The other direction: with nothing signalled yet, the replay is what
        // the child is waiting for.
        let replayed = dir.path().join("replayed");
        let (mut leader, child) =
            spawn_group_with_child(dir.path(), &term_counting_child(&replayed), 1);
        let leader_pid = leader.id();
        let reaper = tokio::task::spawn_blocking(move || leader.wait());
        let now = tokio::time::Instant::now();
        contain_exited_group(
            leader_pid,
            Some((
                now + std::time::Duration::from_secs(1),
                now + std::time::Duration::from_secs(60),
            )),
            false,
            true,
        )
        .await;
        assert_eq!(
            std::fs::read_to_string(&replayed).ok().as_deref(),
            Some("x\n"),
            "a group nothing has signalled yet must get the ladder's SIGTERM"
        );
        let _ = reaper.await;
        let _ = child;
    }

    /// …and the wait does not outlive the group: members that shut down on their
    /// own end it, instead of the containment sitting there until the deadline
    /// with a pgid the kernel is free to hand to somebody else.
    #[tokio::test]
    async fn stopped_node_group_wait_ends_as_soon_as_the_group_empties() {
        let dir = tempfile::tempdir().unwrap();
        let (mut leader, child) = spawn_group_with_child(dir.path(), "sleep 1", 1);
        // Reap the leader while the containment waits, as the node's own
        // process-wait task does — an unreaped one lingers as a zombie and keeps
        // the group non-empty forever.
        let leader_pid = leader.id();
        let reaper = tokio::task::spawn_blocking(move || leader.wait());

        let started = tokio::time::Instant::now();
        let now = tokio::time::Instant::now();
        contain_exited_group(
            leader_pid,
            Some((
                now + std::time::Duration::from_secs(120),
                now + std::time::Duration::from_secs(240),
            )),
            false,
            true,
        )
        .await;

        assert!(
            !process_alive(child),
            "the child exited on its own, so the group has nothing left to contain"
        );
        assert!(
            started.elapsed() < std::time::Duration::from_secs(30),
            "the wait must end with the group, not at the 120s deadline (took {:?})",
            started.elapsed()
        );
        let _ = reaper.await;
    }
}
