//! Pending parameter changes (`ParamUpdate` / `ParamDeleted`) are keyed
//! state, not queued events (dora-rs/dora#4143, #4144).
//!
//! Each test fills the ingress channel of a scheduled event stream directly,
//! keeps the ingress sender open, and reads the stream only through its public
//! receive API. Expected sequences: one event per changed key with its newest
//! change, at the position of the first change of that key that arrived while
//! no change of the key was pending; parameter entries and other control
//! events in position order, ahead of inputs; after `Stop`, only the inputs
//! that were already queued.
//!
//! Every test body runs on its own thread with a wall-clock limit, so a read
//! or a wait that hangs fails the test instead of blocking the run.

use super::*;

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicUsize};

/// More pending parameter changes than the 1,000-event control queue holds.
const BACKLOG: usize = NON_INPUT_EVENT_QUEUE_SIZE + 200;

/// The timeout passed to every pattern-aware wait. It is longer than
/// `BODY_LIMIT`, so a wait that runs to its own deadline fails the test.
const WAIT: Duration = Duration::from_secs(120);

/// A wait whose outcome is decided by the events already queued must return
/// within this time.
const PROMPT: Duration = Duration::from_secs(3);

/// A test body that has not finished after this time has hung.
const BODY_LIMIT: Duration = Duration::from_secs(60);

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// Run `body` on its own thread. Fail if it panics, and fail if it does not
/// finish within `BODY_LIMIT` (a read or wait hung while the ingress sender
/// was open).
fn run_bounded(name: &str, body: impl FnOnce() + Send + 'static) {
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    let handle = std::thread::Builder::new()
        .name(name.to_string())
        .spawn(move || {
            body();
            let _ = done_tx.send(());
        })
        .expect("spawn test thread");
    match done_rx.recv_timeout(BODY_LIMIT) {
        Ok(()) => handle.join().expect("test thread"),
        Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => match handle.join() {
            Err(panic) => std::panic::resume_unwind(panic),
            Ok(()) => unreachable!("the test thread ended without reporting"),
        },
        Err(std::sync::mpsc::RecvTimeoutError::Timeout) => panic!(
            "{name}: did not finish within {BODY_LIMIT:?}; a read or a wait hung \
             while the ingress sender was still open"
        ),
    }
}

/// A scheduled event stream whose ingress channel the test fills.
pub(super) struct Harness {
    _node: crate::DoraNode,
    pub(super) events: EventStream,
    tx: tokio::sync::mpsc::Sender<EventItem>,
    capacity: usize,
}

pub(super) fn harness(capacity: usize) -> Harness {
    let (node, mut events) = test_event_stream();
    let (tx, rx) = tokio::sync::mpsc::channel(capacity);
    events.receiver = rx;
    events.use_scheduler = true;
    Harness {
        _node: node,
        events,
        tx,
        capacity,
    }
}

impl Harness {
    pub(super) fn send(&self, events: impl IntoIterator<Item = NodeEvent>) {
        for event in events {
            self.tx
                .try_send(EventItem::NodeEvent { event })
                .expect("the ingress channel has room for the test events");
        }
    }

    /// The number of `dora.non_input_event` drops counted so far.
    pub(super) fn non_input_drops(&mut self) -> u64 {
        let key = DataId::from("dora.non_input_event".to_string());
        self.events
            .drain_drop_counts()
            .get(&key)
            .copied()
            .unwrap_or(0)
    }

    /// Read with `try_recv` until nothing more is ready, and check what was
    /// received and how the reads ended.
    fn expect(&mut self, context: &str, expected: &[Seen], end: End) {
        let (got, actual_end) = read_ready(&mut self.events);
        assert_seq(context, &got, expected);
        assert_eq!(actual_end, end, "{context}: how the reads ended");
    }

    fn assert_drops(&mut self, context: &str, expected: u64) {
        assert_eq!(
            self.non_input_drops(),
            expected,
            "{context}: `dora.non_input_event` drops counted"
        );
    }
}

// ---------------------------------------------------------------------------
// Events as the application sees them
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Cause {
    Manual,
    AllInputsClosed,
}

/// One event as the application reads it.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum Seen {
    Update(String, serde_json::Value),
    Deleted(String),
    Stop(Cause),
    NodeRestarted(String),
    NodeFailed(String),
    Input(String),
    Reload,
    Other(String),
}

impl Seen {
    fn param_key(&self) -> Option<&str> {
        match self {
            Seen::Update(key, _) | Seen::Deleted(key) => Some(key),
            _ => None,
        }
    }
}

pub(super) fn seen(event: Event) -> Seen {
    match event {
        Event::ParamUpdate { key, value } => Seen::Update(key, value),
        Event::ParamDeleted { key } => Seen::Deleted(key),
        Event::Stop(StopCause::Manual) => Seen::Stop(Cause::Manual),
        Event::Stop(StopCause::AllInputsClosed) => Seen::Stop(Cause::AllInputsClosed),
        Event::NodeRestarted { id } => Seen::NodeRestarted(id.to_string()),
        Event::NodeFailed { source_node_id, .. } => Seen::NodeFailed(source_node_id.to_string()),
        Event::Input { id, .. } => Seen::Input(id.to_string()),
        Event::Reload { .. } => Seen::Reload,
        other => Seen::Other(format!("{other:?}")),
    }
}

/// How a read loop ended.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum End {
    /// `try_recv` reported `Empty`: nothing more is ready, the stream is open.
    Empty,
    /// The stream is closed (the read after `Stop` and its queued inputs).
    Closed,
}

/// Read with `try_recv` until nothing more is ready or the stream closes.
pub(super) fn read_ready(events: &mut EventStream) -> (Vec<Seen>, End) {
    let mut got = Vec::new();
    for _ in 0..1_000_000 {
        match events.try_recv() {
            Ok(event) => got.push(seen(event)),
            Err(TryRecvError::Empty) => return (got, End::Empty),
            Err(TryRecvError::Closed) => return (got, End::Closed),
        }
    }
    panic!("try_recv kept returning events; read {} so far", got.len());
}

fn describe(seq: &[Seen], at: usize) -> String {
    let from = at.saturating_sub(2);
    let to = (at + 3).min(seq.len());
    format!("{:?}", &seq[from..to])
}

pub(super) fn assert_seq(context: &str, got: &[Seen], expected: &[Seen]) {
    if got == expected {
        return;
    }
    let first_difference = got
        .iter()
        .zip(expected)
        .position(|(g, e)| g != e)
        .unwrap_or(got.len().min(expected.len()));
    let got_params = got.iter().filter(|s| s.param_key().is_some()).count();
    let expected_params = expected.iter().filter(|s| s.param_key().is_some()).count();
    panic!(
        "{context}: received {} events ({got_params} parameter events), expected {} \
         ({expected_params} parameter events); first difference at index {first_difference}: \
         got {} expected {}",
        got.len(),
        expected.len(),
        describe(got, first_difference),
        describe(expected, first_difference),
    );
}

// ---------------------------------------------------------------------------
// Parameter changes and the specification model
// ---------------------------------------------------------------------------

/// One parameter change: an update (`Some(value)`) or a deletion (`None`).
#[derive(Debug, Clone)]
pub(super) struct Change {
    pub(super) key: String,
    pub(super) value: Option<serde_json::Value>,
}

pub(super) fn set(key: impl Into<String>, value: impl Into<serde_json::Value>) -> Change {
    Change {
        key: key.into(),
        value: Some(value.into()),
    }
}

pub(super) fn del(key: impl Into<String>) -> Change {
    Change {
        key: key.into(),
        value: None,
    }
}

impl Change {
    pub(super) fn event(&self) -> NodeEvent {
        match &self.value {
            Some(value) => NodeEvent::ParamUpdate {
                key: self.key.clone(),
                value_json: serde_json::to_vec(value).unwrap(),
            },
            None => NodeEvent::ParamDeleted {
                key: self.key.clone(),
            },
        }
    }

    pub(super) fn seen(&self) -> Seen {
        match &self.value {
            Some(value) => Seen::Update(self.key.clone(), value.clone()),
            None => Seen::Deleted(self.key.clone()),
        }
    }
}

pub(super) fn events_of(changes: &[Change]) -> Vec<NodeEvent> {
    changes.iter().map(Change::event).collect()
}

/// For changes that are all pending at the same time: one event per key, at
/// the position of the key's first change, carrying its last change.
fn coalesce(changes: &[Change]) -> Vec<Seen> {
    let mut order: Vec<String> = Vec::new();
    let mut last: HashMap<String, &Change> = HashMap::new();
    for change in changes {
        if last.insert(change.key.clone(), change).is_none() {
            order.push(change.key.clone());
        }
    }
    order.iter().map(|key| last[key].seen()).collect()
}

pub(super) fn key(prefix: &str, i: usize) -> String {
    format!("{prefix}{i:06}")
}

/// `count` distinct keys, updates (even index) and deletions (odd index).
pub(super) fn distinct_mixed(prefix: &str, count: usize) -> Vec<Change> {
    (0..count)
        .map(|i| {
            if i % 2 == 0 {
                set(key(prefix, i), i)
            } else {
                del(key(prefix, i))
            }
        })
        .collect()
}

/// `count` distinct keys, updates and deletions mixed, followed by further
/// changes of some keys while their first change is still pending: every 5th
/// key flips between update and deletion, every 15th key is updated again.
fn backlog_with_repeats(prefix: &str, count: usize) -> Vec<Change> {
    let mut changes = distinct_mixed(prefix, count);
    for i in (0..count).step_by(5) {
        changes.push(if i % 2 == 0 {
            del(key(prefix, i))
        } else {
            set(key(prefix, i), 1_000_000 + i)
        });
    }
    for i in (0..count).step_by(15) {
        changes.push(set(key(prefix, i), 2_000_000 + i));
    }
    changes
}

fn tick() -> NodeEvent {
    NodeEvent::Input {
        id: "tick".into(),
        metadata: Arc::new(make_metadata(MetadataParameters::new())),
        data: None,
    }
}

fn tick_seen() -> Seen {
    Seen::Input("tick".to_string())
}

fn restarted(node: &str) -> NodeEvent {
    NodeEvent::NodeRestarted {
        id: NodeId::from(node.to_string()),
    }
}

pub(super) fn reload() -> NodeEvent {
    NodeEvent::Reload { operator_id: None }
}

pub(super) fn calc() -> NodeId {
    NodeId::from("calc".to_string())
}

fn calc_failed_seen() -> Seen {
    Seen::NodeFailed("calc".to_string())
}

/// One small scenario: the events sent, then the sequence and end of the reads.
struct Case {
    name: &'static str,
    sent: Vec<NodeEvent>,
    expected: Vec<Seen>,
    end: End,
}

fn case(name: &'static str, sent: Vec<NodeEvent>, expected: Vec<Seen>, end: End) -> Case {
    Case {
        name,
        sent,
        expected,
        end,
    }
}

/// Run every case on a fresh stream; a failure names the case.
fn check_cases(test: &'static str, build: impl FnOnce() -> Vec<Case> + Send + 'static) {
    run_bounded(test, move || {
        for case in build() {
            let mut h = harness(case.sent.len() + 16);
            h.send(case.sent);
            h.expect(case.name, &case.expected, case.end);
        }
    });
}

// ---------------------------------------------------------------------------
// S1 — no loss
// ---------------------------------------------------------------------------

/// `NON_INPUT_EVENT_QUEUE_SIZE + 1000` distinct keys, updates and deletions
/// mixed (some keys changed again while pending), then a live `watermark`
/// parameter. Every backlog key arrives before the watermark, the application
/// view of every key is its last change, and no control-event drop is counted.
#[test]
fn s1_backlog_beyond_control_queue_then_watermark_delivers_every_key_first() {
    run_bounded("s1_watermark", || {
        let count = NON_INPUT_EVENT_QUEUE_SIZE + 1000;
        let backlog = backlog_with_repeats("p", count);
        let mut h = harness(backlog.len() + 16);
        h.send(events_of(&backlog));
        h.send([set("watermark", "done").event()]);

        let mut before_watermark = Vec::new();
        let mut watermark_seen = false;
        for _ in 0..backlog.len() + 16 {
            let event = seen(h.events.recv().expect("stream open"));
            if event.param_key() == Some("watermark") {
                watermark_seen = true;
                break;
            }
            before_watermark.push(event);
        }
        assert!(watermark_seen, "the watermark was never received");

        // The application view of a key is the last parameter event received.
        let mut view: HashMap<String, Seen> = HashMap::new();
        for event in &before_watermark {
            let key = event
                .param_key()
                .unwrap_or_else(|| panic!("unexpected non-parameter event {event:?}"));
            view.insert(key.to_string(), event.clone());
        }
        let last: HashMap<&String, Seen> = backlog.iter().map(|c| (&c.key, c.seen())).collect();
        let wrong: Vec<_> = last
            .iter()
            .filter(|(key, want)| view.get(**key) != Some(*want))
            .map(|(key, want)| (key, want, view.get(*key)))
            .collect();
        assert!(
            wrong.is_empty(),
            "{} of {count} backlog keys were not received before the watermark or end with a \
             view other than their last change, e.g. {:?}",
            wrong.len(),
            wrong.iter().take(3).collect::<Vec<_>>()
        );

        h.expect("after the watermark", &[], End::Empty);
        h.assert_drops("after the watermark", 0);
    });
}

/// An input is queued ahead of a parameter backlog larger than the control
/// queue. Every parameter event comes first, then the input.
#[test]
fn s1_s3_parameter_backlog_keeps_priority_over_queued_input() {
    run_bounded("s1_s3_priority", || {
        let backlog = backlog_with_repeats("p", NON_INPUT_EVENT_QUEUE_SIZE + 500);
        let mut h = harness(backlog.len() + 16);
        h.send([tick()]);
        h.send(events_of(&backlog));

        let mut expected = coalesce(&backlog);
        expected.push(tick_seen());
        h.expect(
            "parameter backlog ahead of the input",
            &expected,
            End::Empty,
        );
        h.assert_drops("parameter backlog ahead of the input", 0);
    });
}

// ---------------------------------------------------------------------------
// S2 — one event per key
// ---------------------------------------------------------------------------

#[test]
fn s2_pending_changes_of_one_key_are_one_event_with_newest_change() {
    check_cases("s2_coalesce", || {
        let updates: Vec<Change> = (0..100_000).map(|i| set("k", i)).collect();
        vec![
            case(
                "k=1, k=2, k=3",
                events_of(&[set("k", 1), set("k", 2), set("k", 3)]),
                vec![set("k", 3).seen()],
                End::Empty,
            ),
            case(
                "k=1, delete k",
                events_of(&[set("k", 1), del("k")]),
                vec![del("k").seen()],
                End::Empty,
            ),
            case(
                "delete k, k=5",
                events_of(&[del("k"), set("k", 5)]),
                vec![set("k", 5).seen()],
                End::Empty,
            ),
            case(
                "100,000 updates of k",
                events_of(&updates),
                vec![set("k", 99_999).seen()],
                End::Empty,
            ),
        ]
    });
}

/// A change that arrives after the application received the key is a new
/// event; a received change is never delivered again.
#[test]
fn s2_change_after_delivery_is_new_event_and_received_change_never_repeats() {
    run_bounded("s2_after_delivery", || {
        let mut h = harness(64);
        h.send([set("k", 1).event(), set("k", 2).event()]);
        h.expect("k=1, k=2 pending", &[set("k", 2).seen()], End::Empty);

        h.send([set("k", 3).event()]);
        h.expect(
            "k=3 after k=2 was received",
            &[set("k", 3).seen()],
            End::Empty,
        );
        h.expect("nothing new", &[], End::Empty);
    });
}

// ---------------------------------------------------------------------------
// S3 — order
// ---------------------------------------------------------------------------

#[test]
fn s3_parameter_entries_and_control_events_follow_positions() {
    check_cases("s3_positions", || {
        vec![
            case(
                "a=1, b=1, a=2",
                events_of(&[set("a", 1), set("b", 1), set("a", 2)]),
                vec![set("a", 2).seen(), set("b", 1).seen()],
                End::Empty,
            ),
            case(
                "a=1, NodeRestarted x, b=1",
                vec![set("a", 1).event(), restarted("x"), set("b", 1).event()],
                vec![
                    set("a", 1).seen(),
                    Seen::NodeRestarted("x".into()),
                    set("b", 1).seen(),
                ],
                End::Empty,
            ),
            case(
                "a=1, NodeRestarted x, b=1, a=2",
                vec![
                    set("a", 1).event(),
                    restarted("x"),
                    set("b", 1).event(),
                    set("a", 2).event(),
                ],
                vec![
                    set("a", 2).seen(),
                    Seen::NodeRestarted("x".into()),
                    set("b", 1).seen(),
                ],
                End::Empty,
            ),
            case(
                "tick, a=1",
                vec![tick(), set("a", 1).event()],
                vec![set("a", 1).seen(), tick_seen()],
                End::Empty,
            ),
        ]
    });
}

/// A key received earlier takes a new position with its next change.
/// `a=1, b=1` pending, `a=1` read, then `a=2` arrives ⇒ `b=1`, then `a=2`.
#[test]
fn s3_key_received_earlier_takes_new_position() {
    run_bounded("s3_new_position", || {
        let mut h = harness(64);
        h.send([set("a", 1).event(), set("b", 1).event()]);
        assert_eq!(seen(h.events.recv().unwrap()), set("a", 1).seen());
        h.send([set("a", 2).event()]);
        h.expect(
            "a=2 after a=1 was read",
            &[set("b", 1).seen(), set("a", 2).seen()],
            End::Empty,
        );
    });
}

// ---------------------------------------------------------------------------
// S4 — Stop
// ---------------------------------------------------------------------------

#[test]
fn s4_stop_delivers_pending_keys_with_newest_value_and_only_queued_inputs() {
    check_cases("s4_stop_cases", || {
        let stop = Seen::Stop(Cause::Manual);
        vec![
            case(
                "k=1, Stop, k=2",
                vec![set("k", 1).event(), NodeEvent::Stop, set("k", 2).event()],
                vec![set("k", 2).seen(), stop.clone()],
                End::Closed,
            ),
            case(
                "a=1, b=1, Stop, a=2, c=1",
                vec![
                    set("a", 1).event(),
                    set("b", 1).event(),
                    NodeEvent::Stop,
                    set("a", 2).event(),
                    set("c", 1).event(),
                ],
                vec![set("a", 2).seen(), set("b", 1).seen(), stop.clone()],
                End::Closed,
            ),
            case(
                "Stop, k=1",
                vec![NodeEvent::Stop, set("k", 1).event()],
                vec![stop.clone()],
                End::Closed,
            ),
            case(
                "tick, Stop, k=1, Reload",
                vec![tick(), NodeEvent::Stop, set("k", 1).event(), reload()],
                vec![stop, tick_seen()],
                End::Closed,
            ),
            case(
                "tick, tick, a=1, AllInputsClosed",
                vec![
                    tick(),
                    tick(),
                    set("a", 1).event(),
                    NodeEvent::AllInputsClosed,
                ],
                vec![
                    set("a", 1).seen(),
                    tick_seen(),
                    tick_seen(),
                    Seen::Stop(Cause::AllInputsClosed),
                ],
                End::Closed,
            ),
        ]
    });
}

/// A parameter backlog larger than the control queue, then `Stop` ⇒ every key
/// once with its newest change, then `Stop`, then the stream ends. No drop.
#[test]
fn s4_stop_behind_parameter_backlog_comes_after_every_key() {
    run_bounded("s4_stop_behind_backlog", || {
        let backlog = backlog_with_repeats("p", NON_INPUT_EVENT_QUEUE_SIZE + 500);
        let mut h = harness(backlog.len() + 16);
        h.send(events_of(&backlog));
        h.send([NodeEvent::Stop]);
        let mut expected = coalesce(&backlog);
        expected.push(Seen::Stop(Cause::Manual));
        h.expect("backlog, Stop", &expected, End::Closed);
        h.assert_drops("backlog, Stop", 0);
    });
}

/// `Stop`, then a parameter flood larger than the control queue ⇒ `Stop`
/// alone, then the stream ends. The parameters cause no counted drop.
#[test]
fn s4_stop_then_parameter_flood_delivers_stop_alone_without_drops() {
    run_bounded("s4_stop_then_flood", || {
        let flood = distinct_mixed("p", BACKLOG);
        let mut h = harness(flood.len() + 16);
        h.send([NodeEvent::Stop]);
        h.send(events_of(&flood));
        h.expect("Stop, flood", &[Seen::Stop(Cause::Manual)], End::Closed);
        h.assert_drops("Stop, flood", 0);
    });
}

/// Three inputs, then a parameter backlog larger than the control queue, then
/// `AllInputsClosed` ⇒ every key, the three inputs, then the `Stop`.
#[test]
fn s4_all_inputs_closed_stop_after_inputs_and_parameter_backlog() {
    run_bounded("s4_all_inputs_closed_backlog", || {
        let backlog = backlog_with_repeats("p", NON_INPUT_EVENT_QUEUE_SIZE + 50);
        let mut h = harness(backlog.len() + 16);
        h.send([tick(), tick(), tick()]);
        h.send(events_of(&backlog));
        h.send([NodeEvent::AllInputsClosed]);
        let mut expected = coalesce(&backlog);
        expected.extend([tick_seen(), tick_seen(), tick_seen()]);
        expected.push(Seen::Stop(Cause::AllInputsClosed));
        h.expect("inputs, backlog, AllInputsClosed", &expected, End::Closed);
    });
}

/// `Stop` survives a control-queue overflow, which is still counted.
#[test]
fn s4_stop_survives_control_queue_overflow() {
    run_bounded("s4_stop_survives", || {
        let reloads = NON_INPUT_EVENT_QUEUE_SIZE + 200;
        let mut h = harness(reloads + 16);
        h.send([NodeEvent::Stop]);
        h.send((0..reloads).map(|_| reload()));
        h.expect(
            "Stop, Reload flood",
            &[Seen::Stop(Cause::Manual)],
            End::Closed,
        );
        h.assert_drops(
            "Stop, Reload flood",
            (reloads + 1 - NON_INPUT_EVENT_QUEUE_SIZE) as u64,
        );
    });
}

/// A key pending at `Stop` and a continuous stream of parameter changes after
/// `Stop` (to that key and to new keys). `Stop` is still delivered, after
/// exactly one event for the pending key and no event for a new key, and the
/// stream then ends.
#[test]
fn s4_stop_is_not_postponed_by_parameter_stream_after_it() {
    run_bounded("s4_stream_after_stop", || {
        let mut h = harness(4096);
        h.send([set("k", 0).event(), NodeEvent::Stop]);

        let stop_flag = Arc::new(AtomicBool::new(false));
        let sent = Arc::new(AtomicUsize::new(0));
        let producer = {
            let tx = h.tx.clone();
            let stop_flag = stop_flag.clone();
            let sent = sent.clone();
            std::thread::spawn(move || {
                let mut i = 1usize;
                while !stop_flag.load(Ordering::SeqCst) {
                    let change = if i.is_multiple_of(2) {
                        set("k", i)
                    } else {
                        set(format!("n{i}"), i)
                    };
                    let item = EventItem::NodeEvent {
                        event: change.event(),
                    };
                    match tx.try_send(item) {
                        Ok(()) => {
                            sent.fetch_add(1, Ordering::SeqCst);
                        }
                        Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => {
                            std::thread::yield_now();
                        }
                        Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => return,
                    }
                    i += 1;
                    std::thread::sleep(Duration::from_micros(10));
                }
            })
        };
        // Let changes arrive after `Stop` before the first read.
        let start = std::time::Instant::now();
        while sent.load(Ordering::SeqCst) < 50 {
            assert!(
                start.elapsed() < Duration::from_secs(10),
                "the producer did not start"
            );
            std::thread::yield_now();
        }

        let mut before_stop = Vec::new();
        let mut stop_seen = false;
        for _ in 0..100_000 {
            match h.events.recv().map(seen) {
                Some(Seen::Stop(Cause::Manual)) => {
                    stop_seen = true;
                    break;
                }
                Some(other) => before_stop.push(other),
                None => break,
            }
        }
        let after = h.events.recv().map(seen);
        stop_flag.store(true, Ordering::SeqCst);
        producer.join().expect("producer thread");

        assert!(
            stop_seen,
            "Stop was not delivered; read {} events first",
            before_stop.len()
        );
        assert!(
            before_stop.len() == 1 && before_stop[0].param_key() == Some("k"),
            "exactly one event for the key pending at Stop may precede Stop, got {} events, \
             first {:?}",
            before_stop.len(),
            before_stop.first()
        );
        assert_eq!(after, None, "nothing may follow Stop here");
    });
}

// ---------------------------------------------------------------------------
// S5 — bounds and isolation
// ---------------------------------------------------------------------------

/// 999 `NodeRestarted` events interleaved with 2,000 distinct-key changes ⇒
/// all of them, in position order, no drop.
#[test]
fn s5_parameters_do_not_evict_other_control_events() {
    run_bounded("s5_isolation", || {
        let mut sent = Vec::new();
        let mut expected = Vec::new();
        let mut key_index = 0;
        for n in 0..999 {
            let node = format!("n{n:03}");
            sent.push(restarted(&node));
            expected.push(Seen::NodeRestarted(node));
            let changes_here = if n < 2 { 3 } else { 2 };
            for _ in 0..changes_here {
                let change = if key_index % 2 == 0 {
                    set(key("p", key_index), key_index)
                } else {
                    del(key("p", key_index))
                };
                sent.push(change.event());
                expected.push(change.seen());
                key_index += 1;
            }
        }
        assert_eq!(key_index, 2000);

        let mut h = harness(sent.len() + 16);
        h.send(sent);
        h.expect("999 NodeRestarted + 2,000 keys", &expected, End::Empty);
        h.assert_drops("999 NodeRestarted + 2,000 keys", 0);
    });
}

/// 1,025 `Reload` events, then a parameter backlog ⇒ the oldest 25 `Reload`
/// events are evicted and counted, every parameter key is delivered.
#[test]
fn s5_other_control_events_keep_bound_beside_parameter_backlog() {
    run_bounded("s5_bound_kept", || {
        let reloads = NON_INPUT_EVENT_QUEUE_SIZE + 25;
        let backlog = backlog_with_repeats("p", NON_INPUT_EVENT_QUEUE_SIZE + 500);
        let mut h = harness(reloads + backlog.len() + 16);
        h.send((0..reloads).map(|_| reload()));
        h.send(events_of(&backlog));
        let mut expected = vec![Seen::Reload; NON_INPUT_EVENT_QUEUE_SIZE];
        expected.extend(coalesce(&backlog));
        h.expect("1,025 Reload, backlog", &expected, End::Empty);
        h.assert_drops("1,025 Reload, backlog", 25);
    });
}

// ---------------------------------------------------------------------------
// S6 — ingress is never stalled
// ---------------------------------------------------------------------------

/// One `recv()` or `try_recv()` takes every waiting event out of the ingress
/// channel, whatever the number of pending parameter changes.
#[test]
fn s6_one_read_empties_the_ingress_channel() {
    run_bounded("s6_ingress", || {
        let first = distinct_mixed("p", 3 * NON_INPUT_EVENT_QUEUE_SIZE);
        let mut h = harness(first.len() + 16);
        h.send(events_of(&first));
        assert!(h.events.recv().is_some());
        assert_eq!(
            h.tx.capacity(),
            h.capacity,
            "recv() must take every waiting event out of the ingress channel"
        );

        // Repeat and new keys while the first batch is still pending.
        let second: Vec<Change> = (0..3 * NON_INPUT_EVENT_QUEUE_SIZE)
            .map(|i| set(key(if i % 2 == 0 { "p" } else { "q" }, i), i + 7))
            .collect();
        h.send(events_of(&second));
        assert!(h.events.try_recv().is_ok());
        assert_eq!(
            h.tx.capacity(),
            h.capacity,
            "try_recv() must take every waiting event out of the ingress channel"
        );
    });
}

// ---------------------------------------------------------------------------
// S7 — pattern-aware waits
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub(super) enum Wait {
    Service,
    Action,
}

impl Wait {
    pub(super) fn reply(self) -> NodeEvent {
        let parameters = match self {
            Wait::Service => request_id_params("req-1"),
            Wait::Action => goal_params("goal-1", Some(GOAL_STATUS_SUCCEEDED)),
        };
        NodeEvent::Input {
            id: "response".into(),
            metadata: Arc::new(make_metadata(parameters)),
            data: None,
        }
    }

    fn run(self, events: &mut EventStream) -> Result<Event, PatternError> {
        futures::executor::block_on(async {
            match self {
                Wait::Service => events.recv_service_response("req-1", &calc(), WAIT).await,
                Wait::Action => events.recv_action_result("goal-1", &calc(), WAIT).await,
            }
        })
    }

    /// Run the wait, and require it to return within `PROMPT`.
    fn run_prompt(self, events: &mut EventStream) -> Result<Event, PatternError> {
        let start = std::time::Instant::now();
        let outcome = self.run(events);
        let elapsed = start.elapsed();
        assert!(
            elapsed < PROMPT,
            "{self:?} wait took {elapsed:?}; its outcome was decided by queued events, so it \
             must return within {PROMPT:?} (its own timeout is {WAIT:?})"
        );
        outcome
    }

    pub(super) fn assert_reply(self, context: &str, outcome: &Result<Event, PatternError>) {
        let (param, value) = match self {
            Wait::Service => (REQUEST_ID, "req-1"),
            Wait::Action => (GOAL_ID, "goal-1"),
        };
        assert!(
            matches!(outcome, Ok(Event::Input { id, metadata, .. })
                if id.as_str() == "response"
                    && dora_message::metadata::get_string_param(&metadata.parameters, param)
                        == Some(value)),
            "{context}: expected the queued reply, got {outcome:?}"
        );
    }
}

fn assert_calc_failed(context: &str, outcome: &Result<Event, PatternError>) {
    assert!(
        matches!(outcome, Err(PatternError::StreamError(msg)) if msg.contains("`calc` failed")),
        "{context}: expected StreamError for the failed server, got {outcome:?}"
    );
}

/// The application reads the server's `NodeFailed` itself, before any wait.
fn read_calc_failure_first(h: &mut Harness) {
    h.send([calc_failed()]);
    let first = h.events.recv().map(seen);
    assert_eq!(
        first,
        Some(calc_failed_seen()),
        "expected the server failure first"
    );
}

/// Scenarios for a wait that passes over a parameter backlog larger than the
/// control queue: (name, the application already read the server's
/// `NodeFailed`, a reply is queued after the backlog, the server's
/// `NodeFailed` is queued after the backlog).
const S7_SCENARIOS: [(&str, bool, bool, bool); 5] = [
    ("reply behind backlog", false, true, false),
    ("reply then failure behind backlog", false, true, true),
    (
        "reply behind backlog, failure already read",
        true,
        true,
        false,
    ),
    ("failure already read, no reply", true, false, false),
    ("failure behind backlog, no reply", false, false, true),
];

/// With a reply queued, the wait returns it. Without one, the server failure
/// is known or queued, and the wait gives `StreamError` promptly. Afterwards
/// the application receives exactly one event per key in position order, then
/// the queued `NodeFailed` once. No drop is counted.
#[test]
fn s7_waits_pass_over_parameter_backlog_for_service_and_action() {
    run_bounded("s7_backlog_scenarios", || {
        for wait in [Wait::Service, Wait::Action] {
            for (name, failure_read, reply_queued, failure_queued) in S7_SCENARIOS {
                let context = format!("{wait:?} wait, {name}");
                let mut h = harness(4 * BACKLOG);
                if failure_read {
                    read_calc_failure_first(&mut h);
                }
                let backlog = backlog_with_repeats("p", BACKLOG);
                h.send(events_of(&backlog));
                if reply_queued {
                    h.send([wait.reply()]);
                }
                if failure_queued {
                    h.send([calc_failed()]);
                }

                if reply_queued {
                    wait.assert_reply(&context, &wait.run(&mut h.events));
                } else {
                    assert_calc_failed(&context, &wait.run_prompt(&mut h.events));
                }

                let mut expected = coalesce(&backlog);
                if failure_queued {
                    expected.push(calc_failed_seen());
                }
                h.expect(&format!("{context}: after the wait"), &expected, End::Empty);
                h.assert_drops(&context, 0);
            }
        }
    });
}

/// A wait passes over a parameter backlog. Then every third key changes again.
/// Each key is delivered once, at its first position, with its newest change.
#[test]
fn s7_key_changed_again_after_wait_is_delivered_once_with_newest_value() {
    run_bounded("s7_changed_after_wait", || {
        let mut h = harness(4 * BACKLOG);
        let backlog = backlog_with_repeats("p", BACKLOG);
        h.send(events_of(&backlog));
        h.send([Wait::Service.reply()]);
        Wait::Service.assert_reply("wait", &Wait::Service.run(&mut h.events));

        let again: Vec<Change> = (0..BACKLOG)
            .step_by(3)
            .map(|i| set(key("p", i), 3_000_000 + i))
            .collect();
        h.send(events_of(&again));

        let all: Vec<Change> = backlog.iter().chain(&again).cloned().collect();
        h.expect("changes after the wait", &coalesce(&all), End::Empty);
    });
}

/// The reply is queued behind a parameter backlog and `Stop`. The wait returns
/// it; afterwards every key, then `Stop`, then the stream ends.
#[test]
fn s7_reply_behind_parameter_backlog_and_stop_is_found() {
    run_bounded("s7_reply_behind_stop_backlog", || {
        let mut h = harness(4 * BACKLOG);
        let backlog = backlog_with_repeats("p", BACKLOG);
        h.send(events_of(&backlog));
        h.send([NodeEvent::Stop, Wait::Service.reply()]);
        Wait::Service.assert_reply("wait", &Wait::Service.run(&mut h.events));

        let mut expected = coalesce(&backlog);
        expected.push(Seen::Stop(Cause::Manual));
        h.expect("after the wait", &expected, End::Closed);
    });
}

/// 1,100 `Reload` events and a parameter backlog are queued ahead of the
/// reply. Ordinary control events keep the 1,000 bound (100 drops counted);
/// every parameter key is delivered after the wait.
#[test]
fn s7_passthrough_keeps_control_bound_beside_parameter_backlog() {
    run_bounded("s7_passthrough_bound", || {
        let reloads = NON_INPUT_EVENT_QUEUE_SIZE + 100;
        let mut h = harness(reloads + 4 * BACKLOG);
        let backlog = backlog_with_repeats("p", BACKLOG + 300);
        h.send((0..reloads).map(|_| reload()));
        h.send(events_of(&backlog));
        h.send([Wait::Service.reply()]);
        Wait::Service.assert_reply("wait", &Wait::Service.run(&mut h.events));

        let mut expected = vec![Seen::Reload; NON_INPUT_EVENT_QUEUE_SIZE];
        expected.extend(coalesce(&backlog));
        h.expect("after the wait", &expected, End::Empty);
        h.assert_drops("after the wait", 100);
    });
}

/// `Stop` is never evicted from the events a wait passes over, and the
/// control-event overflow is counted.
#[test]
fn s7_passthrough_never_evicts_stop() {
    run_bounded("s7_stop_kept", || {
        let reloads = NON_INPUT_EVENT_QUEUE_SIZE + 200;
        let mut h = harness(reloads + 16);
        h.send((0..reloads).map(|_| reload()));
        h.send([NodeEvent::Stop, Wait::Service.reply()]);
        Wait::Service.assert_reply("wait", &Wait::Service.run(&mut h.events));

        let mut expected = vec![Seen::Reload; NON_INPUT_EVENT_QUEUE_SIZE - 1];
        expected.push(Seen::Stop(Cause::Manual));
        h.expect("after the wait", &expected, End::Closed);
        h.assert_drops(
            "after the wait",
            (reloads + 1 - NON_INPUT_EVENT_QUEUE_SIZE) as u64,
        );
    });
}

/// A reply queued ahead of `Stop` is found; `Stop` then reaches the
/// application, and the stream ends.
#[test]
fn s7_reply_queued_ahead_of_stop_is_found() {
    run_bounded("s7_reply_then_stop", || {
        let mut h = harness(64);
        h.send([Wait::Service.reply(), NodeEvent::Stop]);
        Wait::Service.assert_reply("wait", &Wait::Service.run(&mut h.events));
        h.expect("after the wait", &[Seen::Stop(Cause::Manual)], End::Closed);
    });
}

/// A wait after `Stop` without a reply gives `StreamEnded`.
#[test]
fn s7_wait_after_stop_without_reply_is_stream_ended() {
    run_bounded("s7_stream_ended", || {
        let mut h = harness(64);
        h.send([NodeEvent::Stop]);
        assert_eq!(h.events.recv().map(seen), Some(Seen::Stop(Cause::Manual)));
        let outcome = Wait::Service.run_prompt(&mut h.events);
        assert!(
            matches!(outcome, Err(PatternError::StreamEnded)),
            "expected StreamEnded, got {outcome:?}"
        );
        h.expect("after the wait", &[], End::Closed);
    });
}

// ---------------------------------------------------------------------------
// S8 — read paths agree
// ---------------------------------------------------------------------------

/// The same pending events for every read path: coalescing changes, a control
/// event, a deletion, a distinct-key backlog larger than the control queue,
/// and a final input. Returns the events to send and the sequence the
/// application must receive.
fn s8_scenario() -> (Vec<NodeEvent>, Vec<Seen>) {
    let head = [set("a", 1), set("b", 1), set("a", 2)];
    let tail = [set("c", 1), del("b")];
    let backlog = distinct_mixed("q", BACKLOG);

    let mut sent: Vec<NodeEvent> = events_of(&head);
    sent.push(restarted("x"));
    sent.extend(events_of(&tail));
    sent.extend(events_of(&backlog));
    sent.push(tick());

    let mut expected = vec![
        set("a", 2).seen(),
        del("b").seen(),
        Seen::NodeRestarted("x".into()),
        set("c", 1).seen(),
    ];
    expected.extend(coalesce(&backlog));
    expected.push(tick_seen());
    (sent, expected)
}

#[derive(Debug, Clone, Copy)]
enum ReadPath {
    Recv,
    RecvTimeout,
    TryRecv,
    Drain,
    RecvAsync,
}

/// Read the S8 scenario with `path` until the final input arrives (or nothing
/// more is ready, for the non-blocking paths).
fn read_with(path: ReadPath, events: &mut EventStream, limit: usize) -> Vec<Seen> {
    let mut got = Vec::new();
    match path {
        ReadPath::Recv | ReadPath::RecvTimeout | ReadPath::RecvAsync => {
            for _ in 0..limit {
                let event = match path {
                    ReadPath::Recv => events.recv(),
                    ReadPath::RecvTimeout => events.recv_timeout(Duration::from_secs(5)),
                    _ => futures::executor::block_on(events.recv_async()),
                };
                got.push(seen(event.expect("stream open")));
                if got.last() == Some(&tick_seen()) || matches!(got.last(), Some(Seen::Other(_))) {
                    break;
                }
            }
        }
        ReadPath::TryRecv => {
            let (all, end) = read_ready(events);
            assert_eq!(end, End::Empty, "try_recv: how the reads ended");
            got = all;
        }
        ReadPath::Drain => {
            got = events
                .drain()
                .expect("stream open")
                .into_iter()
                .map(seen)
                .collect();
            let again = events.drain().expect("stream open");
            assert!(
                again.is_empty(),
                "a second drain returned {} more events",
                again.len()
            );
        }
    }
    got
}

#[test]
fn s8_every_read_path_gives_the_specified_parameter_sequence() {
    run_bounded("s8_read_paths", || {
        for path in [
            ReadPath::Recv,
            ReadPath::RecvTimeout,
            ReadPath::TryRecv,
            ReadPath::Drain,
            ReadPath::RecvAsync,
        ] {
            let context = format!("{path:?}");
            let (sent, expected) = s8_scenario();
            let mut h = harness(sent.len() + 16);
            h.send(sent);
            let got = read_with(path, &mut h.events, expected.len() + 16);
            assert_seq(&context, &got, &expected);
            h.expect(
                &format!("{context}: after the final input"),
                &[],
                End::Empty,
            );
            h.assert_drops(&context, 0);
        }
    });
}
