//! S9: the cost of pending parameter changes is linear in their number
//! (dora-rs/dora#4144 review).
//!
//! Each test fills the ingress channel of a scheduled event stream directly,
//! keeps the ingress sender open, and uses only the public receive API. The
//! operations under test (a pattern-aware wait, the first read that absorbs a
//! backlog, the reads that deliver it) run as timed phases: each phase must
//! finish within `BOUND`. The sizes make an implementation whose cost is
//! quadratic in the number of pending changes exceed `BOUND` by far more than
//! ten times, while a linear implementation stays well below it.
//!
//! Every test body runs on its own thread. A watcher fails the test as soon as
//! a phase overruns its bound, so a slow or hung phase never blocks the run.

use super::param_state::{
    Change, End, Seen, Wait, assert_seq, calc, del, distinct_mixed, events_of, harness, key,
    read_ready, reload, seen, set,
};
use super::*;

use std::collections::HashMap;
use std::sync::mpsc as std_mpsc;

/// Distinct parameter keys in each backlog.
const KEYS: usize = 200_000;

/// Ordinary control events interleaved with the backlog in the eviction test:
/// one `Reload` after every `KEYS_PER_RELOAD` keys.
const KEYS_PER_RELOAD: usize = 2;

/// Each timed phase must finish within this time.
const BOUND: Duration = Duration::from_secs(5);

/// The timeout passed to every pattern-aware wait: the reply is already
/// queued, so the wait must return it within this time.
const WAIT_TIMEOUT: Duration = BOUND;

/// Untimed work between phases (building and sending the test events,
/// comparing results) must finish within this time.
const UNTIMED_LIMIT: Duration = Duration::from_secs(120);

// ---------------------------------------------------------------------------
// Timed phases on a watched thread
// ---------------------------------------------------------------------------

enum Mark {
    Start(&'static str),
    End,
}

/// Handed to the test body; marks the start and end of each timed phase.
struct Phases {
    marks: std_mpsc::Sender<Mark>,
}

impl Phases {
    /// Run `op` as a timed phase. It must finish within `BOUND`.
    fn timed<T>(&self, name: &'static str, op: impl FnOnce() -> T) -> T {
        self.marks.send(Mark::Start(name)).expect("watcher alive");
        let start = std::time::Instant::now();
        let out = op();
        let elapsed = start.elapsed();
        self.marks.send(Mark::End).expect("watcher alive");
        assert!(
            elapsed < BOUND,
            "{name}: took {elapsed:?}, time bound is {BOUND:?} (cost must be linear in the \
             number of pending parameter changes)"
        );
        out
    }
}

/// Run `body` on its own thread. Fail as soon as a timed phase overruns
/// `BOUND`, when untimed work exceeds `UNTIMED_LIMIT`, or when the body panics.
fn run_phased(name: &str, body: impl FnOnce(&Phases) + Send + 'static) {
    let (marks_tx, marks_rx) = std_mpsc::channel();
    let handle = std::thread::Builder::new()
        .name(name.to_string())
        .spawn(move || body(&Phases { marks: marks_tx }))
        .expect("spawn test thread");
    let mut phase: Option<&'static str> = None;
    loop {
        let limit = if phase.is_some() {
            BOUND
        } else {
            UNTIMED_LIMIT
        };
        match marks_rx.recv_timeout(limit) {
            Ok(Mark::Start(p)) => phase = Some(p),
            Ok(Mark::End) => phase = None,
            Err(std_mpsc::RecvTimeoutError::Disconnected) => match handle.join() {
                Err(panic) => std::panic::resume_unwind(panic),
                Ok(()) => return,
            },
            Err(std_mpsc::RecvTimeoutError::Timeout) => match phase {
                Some(p) => panic!(
                    "{name}: {p}: did not finish within the time bound {BOUND:?} (cost must be \
                     linear in the number of pending parameter changes)"
                ),
                None => panic!("{name}: untimed test work exceeded {UNTIMED_LIMIT:?}"),
            },
        }
    }
}

fn wait_for_reply(events: &mut EventStream) -> Result<Event, PatternError> {
    futures::executor::block_on(events.recv_service_response("req-1", &calc(), WAIT_TIMEOUT))
}

/// Read with `try_recv` until nothing more is ready. The stream must stay open.
fn read_open(events: &mut EventStream) -> Vec<Seen> {
    let (got, end) = read_ready(events);
    assert_eq!(end, End::Empty, "the stream must stay open");
    got
}

/// A second change of every key of the distinct-key backlog, with a new value;
/// keys deleted before are set, keys set before are deleted, every third key
/// gets a new value instead.
fn second_changes() -> Vec<Change> {
    (0..KEYS)
        .map(|i| {
            if i % 2 == 1 || i % 3 == 0 {
                set(key("p", i), 5_000_000 + i)
            } else {
                del(key("p", i))
            }
        })
        .collect()
}

fn seen_of(changes: &[Change]) -> Vec<Seen> {
    changes.iter().map(Change::seen).collect()
}

// ---------------------------------------------------------------------------
// S9 — linear cost
// ---------------------------------------------------------------------------

/// The reply is queued behind `KEYS` distinct-key changes, alone or followed by
/// the expected server's `NodeFailed` (failed-server path). The wait returns
/// the reply within its timeout; the application then reads every key (and the
/// `NodeFailed`), within the time bound.
#[test]
fn s9_wait_over_large_backlog_returns_reply_in_time_then_every_key() {
    for (name, failed_server) in [("server alive", false), ("server failed", true)] {
        run_phased(name, move |phases| {
            let backlog = distinct_mixed("p", KEYS);
            let mut h = harness(KEYS + 16);
            h.send(events_of(&backlog));
            h.send([Wait::Service.reply()]);
            if failed_server {
                h.send([calc_failed()]);
            }

            let outcome = phases.timed("wait over the backlog", || wait_for_reply(&mut h.events));
            Wait::Service.assert_reply(name, &outcome);

            let got = phases.timed("reading the backlog after the wait", || {
                read_open(&mut h.events)
            });
            let mut expected = seen_of(&backlog);
            if failed_server {
                expected.push(Seen::NodeFailed("calc".to_string()));
            }
            assert_seq(name, &got, &expected);
            assert_eq!(
                h.non_input_drops(),
                0,
                "{name}: no control-event drop may be counted"
            );
        });
    }
}

/// `KEYS` distinct-key changes interleaved with `KEYS / KEYS_PER_RELOAD`
/// `Reload` events, far more than `NON_INPUT_EVENT_QUEUE_SIZE`, so the control
/// bound evicts many times. The first read absorbs everything within the time
/// bound. Every key is delivered; only the newest `NON_INPUT_EVENT_QUEUE_SIZE`
/// `Reload` events survive, at their positions, and the evicted ones are
/// counted.
#[test]
fn s9_large_backlog_beside_control_overflow_is_absorbed_in_time() {
    run_phased("s9_control_overflow", |phases| {
        let reloads = KEYS / KEYS_PER_RELOAD;
        assert!(reloads > 10 * NON_INPUT_EVENT_QUEUE_SIZE);
        let evicted = reloads - NON_INPUT_EVENT_QUEUE_SIZE;

        let backlog = distinct_mixed("p", KEYS);
        let mut sent = Vec::with_capacity(KEYS + reloads);
        let mut expected = Vec::with_capacity(KEYS + NON_INPUT_EVENT_QUEUE_SIZE);
        let mut reloads_sent = 0;
        for (i, change) in backlog.iter().enumerate() {
            sent.push(change.event());
            expected.push(change.seen());
            if (i + 1) % KEYS_PER_RELOAD == 0 {
                sent.push(reload());
                if reloads_sent >= evicted {
                    expected.push(Seen::Reload);
                }
                reloads_sent += 1;
            }
        }
        assert_eq!(reloads_sent, reloads);

        let mut h = harness(sent.len() + 16);
        h.send(sent);

        let first = phases.timed("first read absorbing the backlog", || {
            h.events.try_recv().map(seen)
        });
        let first = first.expect("an event is ready");
        let rest = phases.timed("reading the rest of the backlog", || {
            read_open(&mut h.events)
        });

        let mut got = Vec::with_capacity(rest.len() + 1);
        got.push(first);
        got.extend(rest);
        let reloads_got = got.iter().filter(|s| **s == Seen::Reload).count();
        assert_eq!(
            reloads_got, NON_INPUT_EVENT_QUEUE_SIZE,
            "surviving Reload events"
        );
        assert_seq("backlog beside control overflow", &got, &expected);
        assert_eq!(
            h.non_input_drops(),
            evicted as u64,
            "only the Reload overflow may be counted"
        );
    });
}

/// A wait passes over `KEYS` distinct-key changes, so they sit buffered. Then
/// `KEYS` further changes of the same keys arrive. Reading them completes
/// within the time bound, with one event per key carrying its newest change,
/// at the key's first position.
#[test]
fn s9_new_changes_of_keys_buffered_by_a_wait_are_read_in_time() {
    run_phased("s9_changes_after_wait", |phases| {
        let backlog = distinct_mixed("p", KEYS);
        let mut h = harness(2 * KEYS + 16);
        h.send(events_of(&backlog));
        h.send([Wait::Service.reply()]);

        let outcome = phases.timed("wait over the backlog", || wait_for_reply(&mut h.events));
        Wait::Service.assert_reply("wait over the backlog", &outcome);

        let again = second_changes();
        h.send(events_of(&again));

        let got = phases.timed("reading the changed backlog", || read_open(&mut h.events));
        let newest: HashMap<&str, &Change> = again.iter().map(|c| (c.key.as_str(), c)).collect();
        let expected: Vec<Seen> = backlog
            .iter()
            .map(|c| newest[c.key.as_str()].seen())
            .collect();
        assert_seq("changes after the wait", &got, &expected);
        assert_eq!(
            h.non_input_drops(),
            0,
            "no control-event drop may be counted"
        );
    });
}
