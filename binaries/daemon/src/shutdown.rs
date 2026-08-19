//! Reaping node processes when the daemon is torn down.
//!
//! The daemon's stop path is asynchronous: `stop_all` sends the cooperative
//! `Stop` and schedules the grace → SIGTERM → SIGKILL ladder on per-node
//! tasks. That is fine while the daemon keeps running, but `Destroy` means
//! the process is about to end — and once it does, those tasks are never
//! polled again. A node that ignored the cooperative `Stop` is then never
//! signalled at all and survives with `ppid 1`, holding whatever it held:
//! shared memory, GPU contexts, ports (dora-rs/dora#2980).
//!
//! So a destroy must not reply until the daemon's children are gone. The
//! waiting is deliberately *not* done inline in the event handler: a node
//! shutting down cooperatively finishes by asking the daemon to close its
//! outputs and report them done, and those requests are answered by the very
//! event loop that would be blocked. Waiting there deadlocks every
//! well-behaved node against the reaper until the deadline, turning a clean
//! exit into a signal kill. Instead the destroy is left pending, the loop
//! keeps serving nodes, and [`DestroyWait::poll`] is driven by a tick until
//! the nodes are gone or the deadline forces a kill.

use std::time::Duration;
use sysinfo::{Pid, ProcessRefreshKind, ProcessStatus, ProcessesToUpdate, System};
use tokio::time::Instant;

/// How long a destroy waits for nodes to exit before killing them.
///
/// Sized to outlast the stop ladder that the coordinator's `StopDataflow`
/// already scheduled (`DEFAULT_STOP_GRACE` for SIGTERM, half as much again
/// for SIGKILL), so a node that would have died on either rung dies *there*,
/// through the daemon's normal path, which signals the node's whole process
/// group. Killing earlier would cut short the SIGTERM window that nodes use
/// to flush. This deadline is the backstop for whatever that ladder does not
/// finish in time.
pub(crate) const DESTROY_WAIT: Duration = Duration::from_secs(16);

/// How long to wait for a killed process to disappear. SIGKILL cannot be
/// caught, so this only covers the kernel tearing the process down.
const KILL_SETTLE: Duration = Duration::from_secs(2);

/// How often liveness is re-checked while a destroy is pending.
pub(crate) const POLL_INTERVAL: Duration = Duration::from_millis(100);

/// A destroy that is waiting for this daemon's nodes to exit.
pub(crate) struct DestroyWait {
    /// When to stop waiting and kill what is left.
    deadline: Instant,
    /// Set once the kill has been issued; `deadline` then becomes the settle
    /// deadline, and the next expiry gives up rather than killing again.
    killed: bool,
    /// Pids that had to be killed, for the caller to report.
    pub killed_pids: Vec<u32>,
    /// Pids confirmed to be this daemon's children at least once.
    ///
    /// Nodes are spawned as process-group leaders, so a confirmed pid is
    /// also its group's id — which is what lets the group outlive the pid
    /// and still be recognized as ours (#3004 review).
    confirmed: std::collections::HashSet<u32>,
    system: System,
    own_pid: u32,
}

/// What [`DestroyWait::poll`] concluded.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum DestroyProgress {
    /// Nodes are still running; poll again.
    Waiting,
    /// No node processes are left — the daemon may exit.
    Done,
    /// Gave up on these pids: they survived the kill, or could not be
    /// killed. Exiting now orphans them, which is worth an error.
    Abandoned(Vec<u32>),
}

impl DestroyWait {
    pub fn new() -> Self {
        Self {
            deadline: Instant::now() + DESTROY_WAIT,
            killed: false,
            killed_pids: Vec::new(),
            confirmed: std::collections::HashSet::new(),
            system: System::new(),
            own_pid: std::process::id(),
        }
    }

    /// Re-check `pids` and act if the deadline has passed.
    ///
    /// `pids` is re-read from the daemon's state on every call rather than
    /// captured up front, so a node that respawns mid-teardown is followed to
    /// its new process instead of leaving the old pid to be chased.
    pub fn poll(&mut self, pids: &[u32]) -> DestroyProgress {
        let alive = self.live_children(pids);
        if alive.is_empty() {
            return DestroyProgress::Done;
        }
        if Instant::now() < self.deadline {
            return DestroyProgress::Waiting;
        }
        if self.killed {
            return DestroyProgress::Abandoned(alive);
        }

        for pid in &alive {
            if kill_process_group(*pid) {
                self.killed_pids.push(*pid);
            }
        }
        self.killed = true;
        self.deadline = Instant::now() + KILL_SETTLE;
        DestroyProgress::Waiting
    }

    /// The subset of `pids` whose process — or whose process *group* — is
    /// still running.
    ///
    /// Parentage, not mere existence, is what makes a pid ours: once a node
    /// exits its pid is free for reuse, and killing blind would eventually
    /// hit an unrelated process.
    ///
    /// The group half matters because the tracked pid is often a wrapper.
    /// `uv run python node.py` dies to the ladder's SIGTERM while the
    /// interpreter it spawned ignores it; watching only the leader would call
    /// that node gone and let the destroy finish, leaving exactly the orphan
    /// one level down that this module exists to prevent (#3004 review).
    ///
    /// The group is only consulted for a pid confirmed ours while alive, and
    /// only while that pid has not been recycled out from under us. Once a
    /// leader exits, a process group can outlive it and the leader pid is free
    /// for the OS to reuse: if it is recycled into an unrelated process that
    /// leads that same group id, a bare `kill(-pid, 0)` probe would pass and
    /// drag the stranger's group in as ours. See
    /// [`pid_recycled_into_stranger`] for what separates that from the
    /// legitimate remnant (#3067).
    fn live_children(&mut self, pids: &[u32]) -> Vec<u32> {
        if pids.is_empty() {
            return Vec::new();
        }
        let lookup: Vec<Pid> = pids.iter().map(|pid| Pid::from_u32(*pid)).collect();
        self.system.refresh_processes_specifics(
            ProcessesToUpdate::Some(&lookup),
            true,
            ProcessRefreshKind::nothing(),
        );

        let mut alive = Vec::new();
        for pid in pids.iter().copied() {
            let process = self.system.process(Pid::from_u32(pid));
            let leader_alive = process.is_some_and(|process| is_live_child(process, self.own_pid));
            let recycled = pid_recycled_into_stranger(pid, process, self.own_pid);
            if leader_alive {
                self.confirmed.insert(pid);
                alive.push(pid);
            } else if self.confirmed.contains(&pid) && !recycled && process_group_has_members(pid) {
                alive.push(pid);
            }
        }
        alive
    }
}

/// Whether any process is still in the group led by `pid`.
///
/// Signal 0 performs the permission and existence checks without delivering
/// anything, so this asks the kernel the question directly rather than
/// enumerating every process on the machine.
fn process_group_has_members(pid: u32) -> bool {
    #[cfg(unix)]
    {
        // SAFETY: signal 0 delivers nothing; this is a pure existence check.
        unsafe { libc::kill(-(pid as i32), 0) == 0 }
    }
    #[cfg(not(unix))]
    {
        // Windows has no process groups in this sense; the job object the
        // node was spawned into ties its children to the leader's lifetime.
        let _ = pid;
        false
    }
}

/// Kill the node's whole process group, not just the pid the daemon tracks.
///
/// Nodes are spawned as process-group leaders, which is what lets the stop
/// ladder's signals reach whatever the node spawned. The tracked pid is
/// frequently a wrapper — `uv run python script.py`, `sh -c ...` — so
/// signalling it alone would leave the real node behind: the same orphan, one
/// level down.
fn kill_process_group(pid: u32) -> bool {
    #[cfg(unix)]
    {
        // SAFETY: `killpg`/`kill` on a pid this process spawned as a group
        // leader. A pid the OS recycled is excluded upstream in
        // `live_children` — by the parentage check on the leader path and by
        // the recycled-stranger guard on the group-fallback path (#3067).
        let killed = unsafe { libc::killpg(pid as i32, libc::SIGKILL) } == 0;
        if killed {
            return true;
        }
        // Not a group leader after all (the node called `setsid`), or the
        // group is already gone: fall back to the process itself.
        unsafe { libc::kill(pid as i32, libc::SIGKILL) == 0 }
    }
    #[cfg(not(unix))]
    {
        // Windows: nodes are spawned into a job object, which terminates
        // their children along with them.
        let mut system = System::new();
        system.refresh_processes_specifics(
            ProcessesToUpdate::Some(&[Pid::from_u32(pid)]),
            true,
            ProcessRefreshKind::nothing(),
        );
        system
            .process(Pid::from_u32(pid))
            .is_some_and(|process| process.kill())
    }
}

/// Whether `pid` has been recycled by the OS into a process that is not ours
/// *and* leads the group we would otherwise still attribute to the node.
///
/// Only meaningful once the leader has failed [`is_live_child`], which is the
/// one place this is consulted. A process still occupying the pid is then
/// either our own not-yet-reaped zombie leader or a stranger, and parentage —
/// not zombie status — is what separates them: a stranger's unreaped process
/// fails `is_live_child` exactly like our own zombie does, so testing for
/// `!= Zombie` would wave it through. A pid that is simply gone was never
/// recycled at all; its group remnant is genuinely ours.
///
/// A stranger only makes the group ambiguous if it actually leads group `pid`.
/// Most processes inherit their parent's group instead, and nothing can join a
/// group whose leader is gone, so when the new owner sits elsewhere the members
/// the probe found are still our orphans — giving up on them would re-open the
/// leak the fallback exists to close (#3004). Hence both conditions (#3067).
fn pid_recycled_into_stranger(pid: u32, process: Option<&sysinfo::Process>, own_pid: u32) -> bool {
    let Some(process) = process else {
        return false;
    };
    if process.parent() == Some(Pid::from_u32(own_pid)) {
        return false;
    }
    leads_own_group(pid)
}

/// Whether the process at `pid` is the leader of the group with that id.
fn leads_own_group(pid: u32) -> bool {
    #[cfg(unix)]
    {
        // SAFETY: `getpgid` only reads scheduling metadata and cannot affect
        // the target. `-1` (ESRCH — the process vanished under us) is not a
        // match, so a disappearing stranger is not treated as a leader.
        unsafe { libc::getpgid(pid as i32) == pid as i32 }
    }
    #[cfg(not(unix))]
    {
        // No process groups here; `process_group_has_members` already returns
        // false, so the fallback this gates is unreachable.
        let _ = pid;
        false
    }
}

/// A zombie has already exited (its supervision task just has not reaped it
/// yet), and a pid the OS recycled belongs to someone else.
fn is_live_child(process: &sysinfo::Process, own_pid: u32) -> bool {
    if process.status() == ProcessStatus::Zombie {
        return false;
    }
    process.parent() == Some(Pid::from_u32(own_pid))
}

#[cfg(test)]
mod tests {
    use super::*;

    // The `#[cfg(unix)]` tests below drive a real process through the Unix
    // process model: `/bin/sleep`, process groups, `CommandExt::process_group`.
    // They are compiled out rather than `ignore`d because `ignore` only skips
    // a test at *runtime* — the body still has to compile, and the Unix-only
    // APIs it uses do not exist on Windows (dora-rs/dora#2742).
    #[cfg(unix)]
    use std::process::{Command, Stdio};

    #[cfg(unix)]
    fn spawn_sleeper() -> std::process::Child {
        Command::new("sleep")
            .arg("300")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("failed to spawn sleep")
    }

    /// A node that will not exit is killed once the deadline passes — the
    /// case #2980 is about.
    #[cfg(unix)]
    #[tokio::test(start_paused = true)]
    async fn a_node_that_outlives_the_deadline_is_killed() {
        let mut child = spawn_sleeper();
        let pid = child.id();
        let mut wait = DestroyWait::new();

        assert_eq!(
            wait.poll(&[pid]),
            DestroyProgress::Waiting,
            "a live node before the deadline must be waited for, not killed"
        );

        tokio::time::advance(DESTROY_WAIT + Duration::from_secs(1)).await;
        assert_eq!(wait.poll(&[pid]), DestroyProgress::Waiting);
        assert_eq!(wait.killed_pids, vec![pid], "the straggler must be killed");

        let status = child.wait().expect("failed to wait for killed child");
        assert!(!status.success(), "killed process must not exit cleanly");
    }

    /// #3004 review: the tracked pid is often a wrapper. When it exits — to
    /// the ladder's SIGTERM, or on its own — while something it spawned
    /// carries on in its process group, the node is not gone. Following only
    /// the leader would end the destroy there and orphan the child, which is
    /// the very leak this module exists to close, one level down.
    #[cfg(unix)]
    #[tokio::test(start_paused = true)]
    async fn a_wrapper_that_exits_does_not_hide_its_surviving_child() {
        use std::os::unix::process::CommandExt as _;

        // A group leader that spawns a child into its group and exits, like
        // a wrapper dying while the interpreter it launched keeps running.
        //
        // The wrapper blocks on `read` rather than `exit 0` so its lifetime is
        // controlled by the test, not by a race: nothing ordered the first
        // `poll` before a bare `exit 0`, so on a loaded machine the wrapper
        // could already be a reaped-later zombie by then, `is_live_child`
        // returns false, and — being the first poll — the `confirmed`-group
        // fallback does not yet apply, so `poll` returned `Done` and the
        // "wrapper is alive" assertion flaked (#3023). Closing stdin below makes
        // `read` hit EOF, so the wrapper exits exactly where the test wants it.
        let mut wrapper = Command::new("sh")
            .arg("-c")
            .arg("sleep 300 & read line")
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .process_group(0)
            .spawn()
            .expect("failed to spawn wrapper");
        let pid = wrapper.id();

        let mut wait = DestroyWait::new();
        assert_eq!(
            wait.poll(&[pid]),
            DestroyProgress::Waiting,
            "the wrapper is alive, so this is an ordinary wait"
        );

        // Close stdin so the wrapper's `read` hits EOF and it exits; its child
        // lives on. The wrapper is reaped by the `wait()` below.
        drop(wrapper.stdin.take());
        wrapper.wait().expect("failed to wait for wrapper");
        assert_eq!(
            wait.poll(&[pid]),
            DestroyProgress::Waiting,
            "the leader is gone but its group is not — the node is still running"
        );

        tokio::time::advance(DESTROY_WAIT + Duration::from_secs(1)).await;
        assert_eq!(wait.poll(&[pid]), DestroyProgress::Waiting);
        assert_eq!(
            wait.killed_pids,
            vec![pid],
            "the surviving group must be killed"
        );
        // The kill lands on the group, so the child goes with it — once the
        // OS has torn it down and its (re-parented) reaper has collected it,
        // which is what `KILL_SETTLE` covers in production. Real sleeps: the
        // test clock is paused, and this waits on the kernel, not on tokio.
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            match wait.poll(&[pid]) {
                DestroyProgress::Done => break,
                other => assert!(
                    std::time::Instant::now() < deadline,
                    "the killed group never went away: {other:?}"
                ),
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    /// The common case must stay free: a node that exits on its own is never
    /// killed, and the destroy completes as soon as it is gone rather than
    /// sitting out the deadline.
    #[cfg(unix)]
    #[tokio::test(start_paused = true)]
    async fn a_node_that_exits_on_its_own_is_not_killed() {
        let mut child = spawn_sleeper();
        let pid = child.id();
        let mut wait = DestroyWait::new();
        assert_eq!(wait.poll(&[pid]), DestroyProgress::Waiting);

        let _ = child.kill();
        let _ = child.wait();

        assert_eq!(
            wait.poll(&[pid]),
            DestroyProgress::Done,
            "the wait must end when the node does"
        );
        assert!(wait.killed_pids.is_empty(), "nothing to kill");
    }

    /// A pid the OS recycled after the node exited belongs to a stranger.
    /// Killing it would be worse than the leak this module exists to fix.
    #[cfg(unix)]
    #[tokio::test(start_paused = true)]
    async fn a_process_that_is_not_our_child_is_left_alone() {
        let mut child = spawn_sleeper();
        let pid = child.id();

        let mut wait = DestroyWait::new();
        assert_eq!(wait.live_children(&[pid]), vec![pid]);

        let mut foreign = DestroyWait::new();
        foreign.own_pid = std::process::id() + 424_242;
        assert!(
            foreign.live_children(&[pid]).is_empty(),
            "a process parented elsewhere must not be treated as ours"
        );
        tokio::time::advance(DESTROY_WAIT + Duration::from_secs(1)).await;
        assert_eq!(foreign.poll(&[pid]), DestroyProgress::Done);
        assert!(
            foreign.killed_pids.is_empty(),
            "a stranger's pid must never be signalled"
        );

        let _ = child.kill();
        let _ = child.wait();
    }

    /// A group leader that spawns a child into its group, exits, and is left
    /// **unreaped**, so it presents as a zombie rather than disappearing from
    /// the process table. Returns the handle (for cleanup) and its pid.
    ///
    /// Same shape as `a_wrapper_that_exits_does_not_hide_its_surviving_child`,
    /// including the `read`-on-stdin trick that makes the wrapper exit exactly
    /// where the caller wants rather than racing the first poll (#3023).
    #[cfg(unix)]
    fn spawn_unreaped_wrapper() -> (std::process::Child, u32) {
        use std::os::unix::process::CommandExt as _;

        let mut wrapper = Command::new("sh")
            .arg("-c")
            .arg("sleep 300 & read line")
            .stdin(Stdio::piped())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .process_group(0)
            .spawn()
            .expect("failed to spawn wrapper");
        let pid = wrapper.id();

        // Close stdin so `read` hits EOF and the wrapper exits. No `wait()`:
        // without the reap it stays a zombie, which is the state under test.
        // Spin until the kernel has actually gotten there — callers pause the
        // test clock, so this waits on the OS, not on tokio.
        drop(wrapper.stdin.take());
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            let mut system = System::new();
            system.refresh_processes_specifics(
                ProcessesToUpdate::Some(&[Pid::from_u32(pid)]),
                true,
                ProcessRefreshKind::nothing(),
            );
            let zombie = system
                .process(Pid::from_u32(pid))
                .is_some_and(|process| process.status() == ProcessStatus::Zombie);
            if zombie {
                break;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "the wrapper never became a zombie"
            );
            std::thread::sleep(Duration::from_millis(20));
        }
        (wrapper, pid)
    }

    #[cfg(unix)]
    fn kill_group_and_reap(mut wrapper: std::process::Child, pid: u32) {
        // SAFETY: killing the group this test created as a group leader.
        unsafe { libc::killpg(pid as i32, libc::SIGKILL) };
        let _ = wrapper.wait();
    }

    /// One half of the #3067 guard: an *unreaped* leader of ours is not a
    /// recycled pid. It fails `is_live_child` exactly like a stranger would,
    /// so the fallback has to keep trusting its group — otherwise the orphan
    /// #3004 is about slips through whenever the daemon has not gotten around
    /// to reaping the wrapper yet.
    #[cfg(unix)]
    #[tokio::test(start_paused = true)]
    async fn an_unreaped_leader_of_ours_still_covers_its_surviving_child() {
        let (wrapper, pid) = spawn_unreaped_wrapper();

        let mut wait = DestroyWait::new();
        wait.confirmed.insert(pid);
        assert_eq!(
            wait.live_children(&[pid]),
            vec![pid],
            "an unreaped leader of ours is not a recycled pid — its group, \
             which still holds the surviving child, stays ours"
        );

        kill_group_and_reap(wrapper, pid);
    }

    /// The other half: zombie status alone does not make a pid ours. A pid
    /// recycled into a stranger that has itself exited but not been reaped is
    /// a zombie too, and testing for `!= Zombie` would wave its group through
    /// to `killpg`. Parentage is what separates the two cases.
    #[cfg(unix)]
    #[tokio::test(start_paused = true)]
    async fn a_strangers_zombie_at_a_recycled_pid_does_not_make_its_group_ours() {
        let (wrapper, pid) = spawn_unreaped_wrapper();

        // Present the zombie as parented elsewhere, exactly as a pid recycled
        // into a stranger that then exited would. It still leads group `pid`
        // and the group still has a live member, so the bare probe passes and
        // only the parentage check keeps it out.
        let mut wait = DestroyWait::new();
        wait.own_pid = std::process::id() + 424_242;
        wait.confirmed.insert(pid);
        assert!(
            wait.live_children(&[pid]).is_empty(),
            "a zombie parented elsewhere is a recycled pid, not our unreaped \
             leader — its group must not be attributed to us"
        );

        kill_group_and_reap(wrapper, pid);
    }

    /// #3067: once a leader pid is gone we fall back to watching its process
    /// group, but the pid is then free for the OS to reuse. If it is recycled
    /// into a live, unrelated group leader, that stranger's group must not be
    /// attributed to us and killed — even though the pid is still in
    /// `confirmed` and the bare group probe passes.
    #[cfg(unix)]
    #[tokio::test(start_paused = true)]
    async fn a_recycled_leader_pid_does_not_drag_in_a_strangers_group() {
        use std::os::unix::process::CommandExt as _;

        // Spawn a sleeper as its own group leader, so `kill(-pid, 0)` (the
        // group probe) sees a member and the fallback branch is reachable.
        let mut child = Command::new("sleep")
            .arg("300")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .process_group(0)
            .spawn()
            .expect("failed to spawn sleeper");
        let pid = child.id();

        let mut wait = DestroyWait::new();
        assert_eq!(wait.live_children(&[pid]), vec![pid]);
        assert!(
            wait.confirmed.contains(&pid),
            "leader confirmed while alive"
        );

        // Now make the process at `pid` present as a live non-child, exactly
        // as a recycled leader pid would: alive, non-zombie, parented
        // elsewhere. The group probe still passes, so only the recycle guard
        // keeps it out of the live set.
        wait.own_pid = std::process::id() + 424_242;
        assert!(
            wait.live_children(&[pid]).is_empty(),
            "a confirmed leader pid recycled into a live non-child must not \
             drag its group in as ours"
        );

        let _ = child.kill();
        let _ = child.wait();
    }

    /// Nothing to wait for is not a reason to wait.
    #[tokio::test]
    async fn a_daemon_with_no_nodes_finishes_at_once() {
        assert_eq!(DestroyWait::new().poll(&[]), DestroyProgress::Done);
    }
}
