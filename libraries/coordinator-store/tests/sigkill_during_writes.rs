//! Scenario 3.8 of `docs/plan-fault-injection.md`, at the store level: a
//! process writing node params is SIGKILLed between or during commits, and
//! the store is reopened.
//!
//! The writer is this test binary itself, re-run as the ignored `writer_child`
//! test with `WRITER_ENV` set. It sets and deletes params in a fixed order and
//! prints `acked <n>` after each commit returns. The checker kills it after a
//! delay that differs per round, reopens the store, and requires the params
//! and tombstones to be exactly what the acked operations produce, optionally
//! plus the one operation that was in flight.
//!
//! A SIGKILL leaves the OS page cache in place, so this pins atomicity and
//! redb's recovery from an unclean shutdown, not durability against power loss.
#![cfg(all(unix, feature = "redb-backend"))]

use std::{
    collections::{BTreeMap, BTreeSet},
    fmt,
    fs::File,
    io::{BufRead, BufReader, Write},
    os::unix::process::ExitStatusExt,
    path::Path,
    process::{Child, Command, Stdio},
    sync::mpsc::{self, RecvTimeoutError},
    time::Duration,
};

use dora_coordinator_store::{CoordinatorStore, RedbStore};
use dora_message::id::NodeId;
use uuid::Uuid;

const WRITER_ENV: &str = "DORA_STORE_SIGKILL_TEST_WRITER";
const ACK_PREFIX: &str = "acked ";
const ROUNDS: u32 = 40;
const KILL_DELAY_STEP: Duration = Duration::from_micros(1_500);
/// How long the writer may take to open the store and commit once.
const FIRST_ACK_TIMEOUT: Duration = Duration::from_secs(60);
const SIGKILL: i32 = 9;

fn dataflow() -> Uuid {
    Uuid::from_u128(1)
}

fn node() -> NodeId {
    "camera".to_string().into()
}

enum Op {
    Set { key: String, value: Vec<u8> },
    Delete { key: String },
}

/// The `n`th operation the writer performs. Every five operations make one
/// new key go through all the writes the store has: set, delete (writes a
/// tombstone), set again (clears it), overwrite, delete for good.
fn op(n: usize) -> Op {
    let key = format!("k{:05}", n - n % 5);
    match n % 5 {
        1 | 4 => Op::Delete { key },
        _ => Op::Set {
            key,
            value: value(n),
        },
    }
}

fn value(n: usize) -> Vec<u8> {
    vec![(n % 251) as u8; 16 + (n % 9) * 1500]
}

#[derive(Clone, Default, PartialEq)]
struct Params {
    values: BTreeMap<String, Vec<u8>>,
    tombstones: BTreeSet<String>,
}

impl Params {
    /// The state the store must hold after operations `0..n`.
    fn after(n: usize) -> Self {
        let mut state = Self::default();
        (0..n).for_each(|i| state.apply(op(i)));
        state
    }

    /// Mirrors `put_node_param` and `delete_node_param`.
    fn apply(&mut self, op: Op) {
        match op {
            Op::Set { key, value } => {
                self.tombstones.remove(&key);
                self.values.insert(key, value);
            }
            Op::Delete { key } => {
                if self.values.remove(&key).is_some() {
                    self.tombstones.insert(key);
                }
            }
        }
    }

    fn read(store: &RedbStore) -> Self {
        let (dataflow, node) = (dataflow(), node());
        Self {
            values: store
                .list_node_params(&dataflow, &node)
                .expect("list params")
                .into_iter()
                .collect(),
            tombstones: store
                .list_node_param_tombstones(&dataflow, &node)
                .expect("list tombstones")
                .into_iter()
                .map(|(key, _token)| key)
                .collect(),
        }
    }
}

/// Values are a repeated byte, so failure messages show them as `<len>x<byte>`.
fn short(value: &[u8]) -> String {
    format!(
        "{}x{}",
        value.len(),
        value.first().copied().unwrap_or_default()
    )
}

impl fmt::Debug for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Op::Set { key, value } => write!(f, "set {key} = {}", short(value)),
            Op::Delete { key } => write!(f, "delete {key}"),
        }
    }
}

impl fmt::Debug for Params {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let values: BTreeMap<_, _> = self.values.iter().map(|(k, v)| (k, short(v))).collect();
        write!(f, "values {values:?}, tombstones {:?}", self.tombstones)
    }
}

/// The writer process. Ignored, so it runs only when the checker starts it
/// with `--ignored`; it applies operations until it is killed.
#[test]
#[ignore = "started by sigkill_during_param_writes_keeps_every_acked_write"]
fn writer_child() {
    let Some(path) = std::env::var_os(WRITER_ENV) else {
        return;
    };
    let store = RedbStore::open(Path::new(&path)).expect("writer opens store");
    let (dataflow, node) = (dataflow(), node());
    let mut stdout = std::io::stdout().lock();
    for n in 0.. {
        match op(n) {
            Op::Set { key, value } => store.put_node_param(&dataflow, &node, &key, &value),
            Op::Delete { key } => store.delete_node_param(&dataflow, &node, &key),
        }
        .expect("write commits");
        writeln!(stdout, "{ACK_PREFIX}{n}").expect("write ack");
        stdout.flush().expect("flush ack");
    }
}

/// Kills the writer if the checker panics before its own kill, so a failed
/// run does not leave a process writing to the temp dir forever.
struct KillOnDrop(Child);

impl Drop for KillOnDrop {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Counts an `acked <n>` line from the writer. Other lines are libtest's, and
/// the first ack shares its line with libtest's `test writer_child ... `.
fn record_ack(line: &str, acked: &mut usize) {
    if let Some((_, n)) = line.rsplit_once(ACK_PREFIX) {
        let n: usize = n.parse().expect("ack number");
        assert_eq!(n, *acked, "acks arrive in order");
        *acked = n + 1;
    }
}

#[test]
fn sigkill_during_param_writes_keeps_every_acked_write() {
    let dir = tempfile::tempdir().expect("temp dir");
    for round in 0..ROUNDS {
        let path = dir.path().join(format!("round-{round}.redb"));
        let stderr_path = dir.path().join(format!("round-{round}.stderr"));
        let writer_stderr = || std::fs::read_to_string(&stderr_path).unwrap_or_default();
        let mut writer = KillOnDrop(
            Command::new(std::env::current_exe().expect("test binary path"))
                .args(["writer_child", "--exact", "--ignored", "--nocapture"])
                .env(WRITER_ENV, &path)
                .stdout(Stdio::piped())
                .stderr(File::create(&stderr_path).expect("stderr file"))
                .spawn()
                .expect("spawn writer"),
        );
        // Read the writer's output on a thread, so the pipe never fills while
        // the checker sleeps (a full pipe would park the writer between
        // operations) and so waiting for the first ack can time out.
        let stdout = writer.0.stdout.take().expect("piped stdout");
        let (line_tx, lines) = mpsc::channel();
        std::thread::spawn(move || {
            for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                if line_tx.send(line).is_err() {
                    break;
                }
            }
        });

        // Wait for the first commit, so the file exists, then let the writer
        // run for a time that grows with the round. A delay, not an ack count,
        // decides when the kill lands, so it falls at any point inside an
        // operation, not just at its start.
        let mut acked = 0;
        while acked == 0 {
            match lines.recv_timeout(FIRST_ACK_TIMEOUT) {
                Ok(line) => record_ack(&line, &mut acked),
                Err(RecvTimeoutError::Timeout) => panic!(
                    "round {round}: no commit within {FIRST_ACK_TIMEOUT:?}\n{}",
                    writer_stderr()
                ),
                Err(RecvTimeoutError::Disconnected) => panic!(
                    "round {round}: writer exited before its first commit\n{}",
                    writer_stderr()
                ),
            }
        }
        std::thread::sleep(KILL_DELAY_STEP * round);
        writer.0.kill().expect("kill writer");
        let status = writer.0.wait().expect("reap writer");
        assert_eq!(
            status.signal(),
            Some(SIGKILL),
            "round {round}: the writer stopped before the kill ({status})\n{}",
            writer_stderr()
        );
        // Acks the writer printed before the kill landed. The thread ends at
        // the pipe's EOF, which the kill causes.
        lines.iter().for_each(|line| record_ack(&line, &mut acked));

        let store = RedbStore::open(&path)
            .unwrap_or_else(|e| panic!("store must reopen after SIGKILL (round {round}): {e:?}"));
        let found = Params::read(&store);
        // The operation in flight at the kill may or may not have committed;
        // nothing after it was started.
        let expected = Params::after(acked);
        let mut with_in_flight = expected.clone();
        with_in_flight.apply(op(acked));
        assert!(
            found == expected || found == with_in_flight,
            "round {round}: after {acked} acked operations the store holds neither their \
             result nor that plus operation {acked} ({:?})\n\
             found:    {found:?}\nexpected: {expected:?}",
            op(acked),
        );
    }
}
