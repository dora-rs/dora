//! Pending parameter changes (`ParamUpdate` / `ParamDeleted`) are keyed
//! state, not queued events (dora-rs/dora#4143, #4144).
//!
//! Each test fills the ingress channel of a scheduled event stream directly,
//! keeps the ingress sender open, and reads the stream only through its public
//! receive API. Expected sequences follow the specification: one event per
//! changed key with its newest change, at the position of the first change of
//! that key that arrived while no change of the key was pending; parameter
//! entries and other control events in position order, ahead of inputs; after
//! `Stop`, only the inputs that were already queued.
//!
//! Every test body runs on its own thread with a wall-clock limit, so a read
//! or a wait that hangs fails the test instead of blocking the run.

use super::*;

use std::collections::{HashMap, HashSet};
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

/// The key under which control-event drops are counted.
fn non_input_drop_key() -> DataId {
    DataId::from("dora.non_input_event".to_string())
}

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
struct Harness {
    _node: crate::DoraNode,
    events: EventStream,
    tx: tokio::sync::mpsc::Sender<EventItem>,
    capacity: usize,
}

fn harness(capacity: usize) -> Harness {
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
    fn send(&self, events: impl IntoIterator<Item = NodeEvent>) {
        for event in events {
            self.tx
                .try_send(EventItem::NodeEvent { event })
                .expect("the ingress channel has room for the test events");
        }
    }

    fn non_input_drops(&mut self) -> u64 {
        self.events
            .drain_drop_counts()
            .get(&non_input_drop_key())
            .copied()
            .unwrap_or(0)
    }
}

// ---------------------------------------------------------------------------
// Events as the application sees them
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
enum Cause {
    Manual,
    AllInputsClosed,
}

/// One event as the application reads it.
#[derive(Debug, Clone, PartialEq)]
enum Seen {
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

fn seen(event: Event) -> Seen {
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
enum End {
    /// `try_recv` reported `Empty`: nothing more is ready, the stream is open.
    Empty,
    /// The stream is closed (the read after `Stop` and its queued inputs).
    Closed,
}

/// Read with `try_recv` until nothing more is ready or the stream closes.
fn read_ready(events: &mut EventStream) -> (Vec<Seen>, End) {
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

fn assert_seq(context: &str, got: &[Seen], expected: &[Seen]) {
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

fn assert_end(context: &str, got: End, expected: End) {
    assert_eq!(got, expected, "{context}: how the reads ended");
}

// ---------------------------------------------------------------------------
// Parameter changes and the specification model
// ---------------------------------------------------------------------------

/// One parameter change: an update (`Some(value)`) or a deletion (`None`).
#[derive(Debug, Clone)]
struct Change {
    key: String,
    value: Option<serde_json::Value>,
}

fn set(key: impl Into<String>, value: impl Into<serde_json::Value>) -> Change {
    Change {
        key: key.into(),
        value: Some(value.into()),
    }
}

fn del(key: impl Into<String>) -> Change {
    Change {
        key: key.into(),
        value: None,
    }
}

impl Change {
    fn event(&self) -> NodeEvent {
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

    fn seen(&self) -> Seen {
        match &self.value {
            Some(value) => Seen::Update(self.key.clone(), value.clone()),
            None => Seen::Deleted(self.key.clone()),
        }
    }
}

fn events_of(changes: &[Change]) -> Vec<NodeEvent> {
    changes.iter().map(Change::event).collect()
}

/// S2 + S3 for changes that are all pending at the same time: one event per
/// key, at the position of the key's first change, carrying its last change.
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

fn key(prefix: &str, i: usize) -> String {
    format!("{prefix}{i:06}")
}

/// `count` distinct keys, updates (even index) and deletions (odd index).
fn distinct_mixed(prefix: &str, count: usize) -> Vec<Change> {
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

fn reload() -> NodeEvent {
    NodeEvent::Reload { operator_id: None }
}

fn calc() -> NodeId {
    NodeId::from("calc".to_string())
}

fn calc_failed_seen() -> Seen {
    Seen::NodeFailed("calc".to_string())
}

// ---------------------------------------------------------------------------
// S1 — no loss
// ---------------------------------------------------------------------------

/// Formal-model scenario (a): `NON_INPUT_EVENT_QUEUE_SIZE + 1000` distinct keys,
/// updates and deletions mixed (some keys changed again while pending), then a
/// live `watermark` parameter. Every backlog key arrives before the watermark,
/// the application view of every key is its last change, and no control-event
/// drop is counted.
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

        let expected_view: HashMap<String, Seen> = backlog
            .iter()
            .map(|change| (change.key.clone(), change.seen()))
            .collect();
        let mut view: HashMap<String, Seen> = HashMap::new();
        for event in &before_watermark {
            let key = event
                .param_key()
                .unwrap_or_else(|| panic!("unexpected non-parameter event {event:?}"));
            view.insert(key.to_string(), event.clone());
        }
        let missing: Vec<&String> = expected_view
            .keys()
            .filter(|key| !view.contains_key(*key))
            .collect();
        assert!(
            missing.is_empty(),
            "{} of {count} backlog keys were not received before the watermark \
             (received {} parameter events), e.g. {:?}",
            missing.len(),
            before_watermark.len(),
            missing.iter().take(3).collect::<Vec<_>>()
        );
        let wrong: Vec<(&String, &Seen, &Seen)> = expected_view
            .iter()
            .filter(|(key, want)| view.get(*key) != Some(want))
            .map(|(key, want)| (key, want, &view[key]))
            .collect();
        assert!(
            wrong.is_empty(),
            "{} keys end with a view other than their last change, e.g. {:?}",
            wrong.len(),
            wrong.iter().take(3).collect::<Vec<_>>()
        );

        let (rest, end) = read_ready(&mut h.events);
        assert_seq("after the watermark", &rest, &[]);
        assert_end("after the watermark", end, End::Empty);
        assert_eq!(
            h.non_input_drops(),
            0,
            "parameter changes must not cause a `dora.non_input_event` drop"
        );
    });
}

/// S1 + S3: an input is queued ahead of a parameter backlog larger than the
/// control queue. Every parameter event comes first, then the input.
#[test]
fn s1_s3_parameter_backlog_keeps_priority_over_queued_input() {
    run_bounded("s1_s3_priority", || {
        let backlog = backlog_with_repeats("p", NON_INPUT_EVENT_QUEUE_SIZE + 500);
        let mut h = harness(backlog.len() + 16);
        h.send([tick()]);
        h.send(events_of(&backlog));

        let (got, end) = read_ready(&mut h.events);
        let mut expected = coalesce(&backlog);
        expected.push(tick_seen());
        assert_seq("parameter backlog ahead of the input", &got, &expected);
        assert_end("parameter backlog ahead of the input", end, End::Empty);
        assert_eq!(
            h.non_input_drops(),
            0,
            "no control-event drop may be counted"
        );
    });
}

// ---------------------------------------------------------------------------
// S2 — one event per key
// ---------------------------------------------------------------------------

fn check_pending_together(name: &'static str, changes: Vec<Change>, expected: Vec<Seen>) {
    run_bounded(name, move || {
        let mut h = harness(changes.len() + 16);
        h.send(events_of(&changes));
        let (got, end) = read_ready(&mut h.events);
        assert_seq(name, &got, &expected);
        assert_end(name, end, End::Empty);
    });
}

/// `k=1, k=2, k=3` pending ⇒ one `ParamUpdate { k, 3 }`.
#[test]
fn s2_three_updates_of_one_key_are_one_event_with_newest_value() {
    check_pending_together(
        "s2_three_updates",
        vec![set("k", 1), set("k", 2), set("k", 3)],
        vec![set("k", 3).seen()],
    );
}

/// Formal-model scenario (b): `k=1, delete k` pending ⇒ one `ParamDeleted { k }`.
#[test]
fn s2_update_then_delete_is_one_deleted_event() {
    check_pending_together(
        "s2_update_then_delete",
        vec![set("k", 1), del("k")],
        vec![del("k").seen()],
    );
}

/// `delete k, k=5` pending ⇒ one `ParamUpdate { k, 5 }`.
#[test]
fn s2_delete_then_update_is_one_update_event() {
    check_pending_together(
        "s2_delete_then_update",
        vec![del("k"), set("k", 5)],
        vec![set("k", 5).seen()],
    );
}

/// 100,000 updates of one key pending ⇒ one event with the last value.
#[test]
fn s2_hundred_thousand_updates_of_one_key_are_one_event() {
    let changes: Vec<Change> = (0..100_000).map(|i| set("k", i)).collect();
    check_pending_together(
        "s2_hundred_thousand_updates",
        changes,
        vec![set("k", 99_999).seen()],
    );
}

/// Changes pending together are one event; a change that arrives after the
/// application received the key is a new event; a received change is never
/// delivered again.
#[test]
fn s2_change_after_delivery_is_new_event_and_received_change_never_repeats() {
    run_bounded("s2_after_delivery", || {
        let mut h = harness(64);
        h.send([set("k", 1).event(), set("k", 2).event()]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq("k=1, k=2 pending", &got, &[set("k", 2).seen()]);
        assert_end("k=1, k=2 pending", end, End::Empty);

        h.send([set("k", 3).event()]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq("k=3 after k=2 was received", &got, &[set("k", 3).seen()]);
        assert_end("k=3 after k=2 was received", end, End::Empty);
    });
}

/// Guard (unchanged behaviour): one change per read is delivered each time,
/// and a received change is not delivered again.
#[test]
fn guard_s2_change_after_delivery_is_delivered_again() {
    run_bounded("guard_s2_after_delivery", || {
        let mut h = harness(64);
        h.send([set("k", 1).event()]);
        assert_eq!(seen(h.events.recv().unwrap()), set("k", 1).seen());
        let (got, end) = read_ready(&mut h.events);
        assert_seq("after k=1 was received", &got, &[]);
        assert_end("after k=1 was received", end, End::Empty);

        h.send([set("k", 2).event()]);
        assert_eq!(seen(h.events.recv().unwrap()), set("k", 2).seen());
        let (got, end) = read_ready(&mut h.events);
        assert_seq("after k=2 was received", &got, &[]);
        assert_end("after k=2 was received", end, End::Empty);
    });
}

// ---------------------------------------------------------------------------
// S3 — order
// ---------------------------------------------------------------------------

/// Formal-model scenario (e): `a=1, b=1, a=2` pending ⇒ `a=2`, then `b=1`.
#[test]
fn s3_key_keeps_position_of_its_first_pending_change() {
    check_pending_together(
        "s3_first_position",
        vec![set("a", 1), set("b", 1), set("a", 2)],
        vec![set("a", 2).seen(), set("b", 1).seen()],
    );
}

/// `a=1, NodeRestarted x, b=1, a=2` pending ⇒ `a=2`, `NodeRestarted x`, `b=1`.
#[test]
fn s3_parameter_entries_and_control_events_follow_positions() {
    run_bounded("s3_positions_with_control", || {
        let mut h = harness(64);
        h.send([
            set("a", 1).event(),
            restarted("x"),
            set("b", 1).event(),
            set("a", 2).event(),
        ]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq(
            "a=1, NodeRestarted x, b=1, a=2",
            &got,
            &[
                set("a", 2).seen(),
                Seen::NodeRestarted("x".into()),
                set("b", 1).seen(),
            ],
        );
        assert_end("a=1, NodeRestarted x, b=1, a=2", end, End::Empty);
    });
}

/// Guard: `a=1, NodeRestarted x, b=1` pending ⇒ that order.
#[test]
fn guard_s3_parameters_and_control_events_in_arrival_order() {
    run_bounded("guard_s3_arrival_order", || {
        let mut h = harness(64);
        h.send([set("a", 1).event(), restarted("x"), set("b", 1).event()]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq(
            "a=1, NodeRestarted x, b=1",
            &got,
            &[
                set("a", 1).seen(),
                Seen::NodeRestarted("x".into()),
                set("b", 1).seen(),
            ],
        );
        assert_end("a=1, NodeRestarted x, b=1", end, End::Empty);
    });
}

/// Guard: a key received earlier takes a new position with its next change.
/// `a=1, b=1` pending, `a=1` read, then `a=2` arrives ⇒ `b=1`, then `a=2`.
#[test]
fn guard_s3_key_received_earlier_takes_new_position() {
    run_bounded("guard_s3_new_position", || {
        let mut h = harness(64);
        h.send([set("a", 1).event(), set("b", 1).event()]);
        assert_eq!(seen(h.events.recv().unwrap()), set("a", 1).seen());
        h.send([set("a", 2).event()]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq(
            "a=2 after a=1 was read",
            &got,
            &[set("b", 1).seen(), set("a", 2).seen()],
        );
        assert_end("a=2 after a=1 was read", end, End::Empty);
    });
}

/// Guard: a parameter change is served before an input queued ahead of it.
#[test]
fn guard_s3_parameter_served_before_queued_input() {
    run_bounded("guard_s3_priority", || {
        let mut h = harness(64);
        h.send([tick(), set("a", 1).event()]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq("tick, a=1", &got, &[set("a", 1).seen(), tick_seen()]);
        assert_end("tick, a=1", end, End::Empty);
    });
}

// ---------------------------------------------------------------------------
// S4 — Stop
// ---------------------------------------------------------------------------

/// Formal-model scenario (c): `k=1, Stop, k=2` pending ⇒ `k=2`, then `Stop`,
/// then the stream ends.
#[test]
fn s4_key_pending_at_stop_is_delivered_before_stop_with_newest_value() {
    run_bounded("s4_newest_before_stop", || {
        let mut h = harness(64);
        h.send([set("k", 1).event(), NodeEvent::Stop, set("k", 2).event()]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq(
            "k=1, Stop, k=2",
            &got,
            &[set("k", 2).seen(), Seen::Stop(Cause::Manual)],
        );
        assert_end("k=1, Stop, k=2", end, End::Closed);
    });
}

/// `a=1, b=1, Stop, a=2, c=1` pending ⇒ `a=2`, `b=1`, `Stop`; `c` is never
/// delivered.
#[test]
fn s4_only_keys_pending_at_stop_precede_it() {
    run_bounded("s4_only_pending_keys", || {
        let mut h = harness(64);
        h.send([
            set("a", 1).event(),
            set("b", 1).event(),
            NodeEvent::Stop,
            set("a", 2).event(),
            set("c", 1).event(),
        ]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq(
            "a=1, b=1, Stop, a=2, c=1",
            &got,
            &[
                set("a", 2).seen(),
                set("b", 1).seen(),
                Seen::Stop(Cause::Manual),
            ],
        );
        assert_end("a=1, b=1, Stop, a=2, c=1", end, End::Closed);
    });
}

/// Formal-model scenario (d), guard: `Stop, k=1` pending ⇒ `Stop`, and no `k`
/// event ever follows.
#[test]
fn guard_s4_change_arriving_after_stop_is_never_delivered() {
    run_bounded("guard_s4_after_stop", || {
        let mut h = harness(64);
        h.send([NodeEvent::Stop, set("k", 1).event()]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq("Stop, k=1", &got, &[Seen::Stop(Cause::Manual)]);
        assert_end("Stop, k=1", end, End::Closed);
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
        let (got, end) = read_ready(&mut h.events);
        let mut expected = coalesce(&backlog);
        expected.push(Seen::Stop(Cause::Manual));
        assert_seq("backlog, Stop", &got, &expected);
        assert_end("backlog, Stop", end, End::Closed);
        assert_eq!(
            h.non_input_drops(),
            0,
            "no control-event drop may be counted"
        );
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
        let (got, end) = read_ready(&mut h.events);
        assert_seq("Stop, flood", &got, &[Seen::Stop(Cause::Manual)]);
        assert_end("Stop, flood", end, End::Closed);
        assert_eq!(
            h.non_input_drops(),
            0,
            "parameter changes must not cause a `dora.non_input_event` drop"
        );
    });
}

/// Guard: a key pending at `Stop` and a continuous stream of parameter changes
/// after `Stop` (to that key and to new keys). `Stop` is still delivered,
/// after exactly one event for the pending key and no event for a new key, and
/// the stream then ends.
#[test]
fn guard_s4_stop_is_not_postponed_by_parameter_stream_after_it() {
    run_bounded("guard_s4_stream_after_stop", || {
        let capacity = 4096;
        let mut h = harness(capacity);
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
                    let mut item = EventItem::NodeEvent {
                        event: change.event(),
                    };
                    loop {
                        if stop_flag.load(Ordering::SeqCst) {
                            return;
                        }
                        match tx.try_send(item) {
                            Ok(()) => break,
                            Err(tokio::sync::mpsc::error::TrySendError::Full(back)) => {
                                item = back;
                                std::thread::yield_now();
                            }
                            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => return,
                        }
                    }
                    sent.fetch_add(1, Ordering::SeqCst);
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

/// Guard: after `Stop` only the inputs already queued are delivered; control
/// events and parameter changes behind it are not.
#[test]
fn guard_s4_after_stop_only_queued_inputs_follow() {
    run_bounded("guard_s4_inputs_after_stop", || {
        let mut h = harness(64);
        h.send([tick(), NodeEvent::Stop, set("k", 1).event(), reload()]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq(
            "tick, Stop, k=1, Reload",
            &got,
            &[Seen::Stop(Cause::Manual), tick_seen()],
        );
        assert_end("tick, Stop, k=1, Reload", end, End::Closed);
    });
}

/// Guard: the `AllInputsClosed` `Stop` waits for the queued inputs.
#[test]
fn guard_s4_all_inputs_closed_stop_waits_for_queued_inputs() {
    run_bounded("guard_s4_all_inputs_closed", || {
        let mut h = harness(64);
        h.send([
            tick(),
            tick(),
            set("a", 1).event(),
            NodeEvent::AllInputsClosed,
        ]);
        let (got, end) = read_ready(&mut h.events);
        assert_seq(
            "tick, tick, a=1, AllInputsClosed",
            &got,
            &[
                set("a", 1).seen(),
                tick_seen(),
                tick_seen(),
                Seen::Stop(Cause::AllInputsClosed),
            ],
        );
        assert_end("tick, tick, a=1, AllInputsClosed", end, End::Closed);
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
        let (got, end) = read_ready(&mut h.events);
        let mut expected = coalesce(&backlog);
        expected.extend([tick_seen(), tick_seen(), tick_seen()]);
        expected.push(Seen::Stop(Cause::AllInputsClosed));
        assert_seq("inputs, backlog, AllInputsClosed", &got, &expected);
        assert_end("inputs, backlog, AllInputsClosed", end, End::Closed);
    });
}

/// Guard: `Stop` survives a control-queue overflow, which is still counted.
#[test]
fn guard_s4_stop_survives_control_queue_overflow() {
    run_bounded("guard_s4_stop_survives", || {
        let reloads = NON_INPUT_EVENT_QUEUE_SIZE + 200;
        let mut h = harness(reloads + 16);
        h.send([NodeEvent::Stop]);
        h.send((0..reloads).map(|_| reload()));
        let (got, end) = read_ready(&mut h.events);
        assert_seq("Stop, Reload flood", &got, &[Seen::Stop(Cause::Manual)]);
        assert_end("Stop, Reload flood", end, End::Closed);
        assert_eq!(
            h.non_input_drops(),
            (reloads + 1 - NON_INPUT_EVENT_QUEUE_SIZE) as u64,
            "the control-queue overflow must be counted"
        );
    });
}

// ---------------------------------------------------------------------------
// S5 — bounds and isolation
// ---------------------------------------------------------------------------

/// Formal-model scenario (g): 999 `NodeRestarted` events interleaved with
/// 2,000 distinct-key changes ⇒ all of them, in position order, no drop.
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
        let (got, end) = read_ready(&mut h.events);
        assert_seq("999 NodeRestarted + 2,000 keys", &got, &expected);
        assert_end("999 NodeRestarted + 2,000 keys", end, End::Empty);
        assert_eq!(
            h.non_input_drops(),
            0,
            "no control-event drop may be counted"
        );
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
        let (got, end) = read_ready(&mut h.events);
        let mut expected = vec![Seen::Reload; NON_INPUT_EVENT_QUEUE_SIZE];
        expected.extend(coalesce(&backlog));
        assert_seq("1,025 Reload, backlog", &got, &expected);
        assert_end("1,025 Reload, backlog", end, End::Empty);
        assert_eq!(
            h.non_input_drops(),
            25,
            "only the Reload overflow may be counted"
        );
    });
}

/// Guard: more than 1,000 ordinary control events still evict the oldest, and
/// the drops are counted.
#[test]
fn guard_s5_control_queue_overflow_is_counted() {
    run_bounded("guard_s5_overflow", || {
        let reloads = NON_INPUT_EVENT_QUEUE_SIZE + 200;
        let mut h = harness(reloads + 16);
        h.send((0..reloads).map(|_| reload()));
        let (got, end) = read_ready(&mut h.events);
        assert_seq(
            "Reload flood",
            &got,
            &vec![Seen::Reload; NON_INPUT_EVENT_QUEUE_SIZE],
        );
        assert_end("Reload flood", end, End::Empty);
        assert_eq!(h.non_input_drops(), 200);
    });
}

// ---------------------------------------------------------------------------
// S6 — ingress is never stalled
// ---------------------------------------------------------------------------

/// Guard: one `recv()` or `try_recv()` takes every waiting event out of the
/// ingress channel, whatever the number of pending parameter changes.
#[test]
fn guard_s6_one_read_empties_the_ingress_channel() {
    run_bounded("guard_s6_ingress", || {
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
enum Wait {
    Service,
    Action,
}

impl Wait {
    fn reply(self) -> NodeEvent {
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

    fn assert_reply(self, outcome: &Result<Event, PatternError>) {
        let (param, value) = match self {
            Wait::Service => (REQUEST_ID, "req-1"),
            Wait::Action => (GOAL_ID, "goal-1"),
        };
        assert!(
            matches!(outcome, Ok(Event::Input { id, metadata, .. })
                if id.as_str() == "response"
                    && dora_message::metadata::get_string_param(&metadata.parameters, param)
                        == Some(value)),
            "{self:?} wait: expected the queued reply, got {outcome:?}"
        );
    }
}

fn assert_calc_failed(wait: Wait, outcome: &Result<Event, PatternError>) {
    assert!(
        matches!(outcome, Err(PatternError::StreamError(msg)) if msg.contains("`calc` failed")),
        "{wait:?} wait: expected StreamError for the failed server, got {outcome:?}"
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

/// The reply is queued behind a parameter backlog larger than the control
/// queue; the server is alive. The wait returns the reply; afterwards the
/// application receives exactly one event per key, in position order.
fn check_reply_behind_backlog(wait: Wait) {
    let mut h = harness(4 * BACKLOG);
    let backlog = backlog_with_repeats("p", BACKLOG);
    h.send(events_of(&backlog));
    h.send([wait.reply()]);

    let outcome = wait.run(&mut h.events);
    wait.assert_reply(&outcome);

    let (got, end) = read_ready(&mut h.events);
    assert_seq("after the wait", &got, &coalesce(&backlog));
    assert_end("after the wait", end, End::Empty);
    assert_eq!(
        h.non_input_drops(),
        0,
        "no control-event drop may be counted"
    );
}

#[test]
fn s7_service_reply_behind_parameter_backlog_is_found() {
    run_bounded("s7_service_loop", || {
        check_reply_behind_backlog(Wait::Service)
    });
}

#[test]
fn s7_action_result_behind_parameter_backlog_is_found() {
    run_bounded("s7_action_loop", || {
        check_reply_behind_backlog(Wait::Action)
    });
}

/// Formal-model scenario (f): the reply and then the server's `NodeFailed` are
/// queued behind a parameter backlog larger than the control queue. The wait
/// returns the reply; afterwards the application receives one event per key,
/// then the `NodeFailed` once.
fn check_reply_then_failure_behind_backlog(wait: Wait) {
    let mut h = harness(4 * BACKLOG);
    let backlog = backlog_with_repeats("p", BACKLOG);
    h.send(events_of(&backlog));
    h.send([wait.reply(), calc_failed()]);

    let outcome = wait.run(&mut h.events);
    wait.assert_reply(&outcome);

    let (got, end) = read_ready(&mut h.events);
    let mut expected = coalesce(&backlog);
    expected.push(calc_failed_seen());
    assert_seq("after the wait", &got, &expected);
    assert_end("after the wait", end, End::Empty);
    assert_eq!(
        h.non_input_drops(),
        0,
        "no control-event drop may be counted"
    );
}

#[test]
fn s7_service_reply_then_server_failure_behind_backlog_is_found() {
    run_bounded("s7_service_failure_queued", || {
        check_reply_then_failure_behind_backlog(Wait::Service)
    });
}

#[test]
fn s7_action_result_then_server_failure_behind_backlog_is_found() {
    run_bounded("s7_action_failure_queued", || {
        check_reply_then_failure_behind_backlog(Wait::Action)
    });
}

/// The application already read the server's `NodeFailed`. The reply is queued
/// behind a parameter backlog. The wait returns it; afterwards one event per
/// key.
fn check_reply_behind_backlog_after_failure_was_seen(wait: Wait) {
    let mut h = harness(4 * BACKLOG);
    read_calc_failure_first(&mut h);
    let backlog = backlog_with_repeats("p", BACKLOG);
    h.send(events_of(&backlog));
    h.send([wait.reply()]);

    let outcome = wait.run(&mut h.events);
    wait.assert_reply(&outcome);

    let (got, end) = read_ready(&mut h.events);
    assert_seq("after the wait", &got, &coalesce(&backlog));
    assert_end("after the wait", end, End::Empty);
    assert_eq!(
        h.non_input_drops(),
        0,
        "no control-event drop may be counted"
    );
}

#[test]
fn s7_service_reply_behind_backlog_found_after_failure_was_seen() {
    run_bounded("s7_service_failure_seen", || {
        check_reply_behind_backlog_after_failure_was_seen(Wait::Service)
    });
}

#[test]
fn s7_action_result_behind_backlog_found_after_failure_was_seen() {
    run_bounded("s7_action_failure_seen", || {
        check_reply_behind_backlog_after_failure_was_seen(Wait::Action)
    });
}

/// The application already read the server's `NodeFailed`; a parameter backlog
/// follows and no reply. The wait gives `StreamError` promptly; afterwards one
/// event per key.
fn check_known_failure_without_reply(wait: Wait) {
    let mut h = harness(4 * BACKLOG);
    read_calc_failure_first(&mut h);
    let backlog = backlog_with_repeats("p", BACKLOG);
    h.send(events_of(&backlog));

    let outcome = wait.run_prompt(&mut h.events);
    assert_calc_failed(wait, &outcome);

    let (got, end) = read_ready(&mut h.events);
    assert_seq("after the wait", &got, &coalesce(&backlog));
    assert_end("after the wait", end, End::Empty);
    assert_eq!(
        h.non_input_drops(),
        0,
        "no control-event drop may be counted"
    );
}

#[test]
fn s7_service_known_failure_without_reply_is_prompt_and_keeps_backlog() {
    run_bounded("s7_service_known_failure", || {
        check_known_failure_without_reply(Wait::Service)
    });
}

#[test]
fn s7_action_known_failure_without_result_is_prompt_and_keeps_backlog() {
    run_bounded("s7_action_known_failure", || {
        check_known_failure_without_reply(Wait::Action)
    });
}

/// The server's `NodeFailed` is queued behind a parameter backlog, with no
/// reply. The wait gives `StreamError` promptly; afterwards one event per key,
/// then the `NodeFailed` once.
fn check_failure_behind_backlog_without_reply(wait: Wait) {
    let mut h = harness(4 * BACKLOG);
    let backlog = backlog_with_repeats("p", BACKLOG);
    h.send(events_of(&backlog));
    h.send([calc_failed()]);

    let outcome = wait.run_prompt(&mut h.events);
    assert_calc_failed(wait, &outcome);

    let (got, end) = read_ready(&mut h.events);
    let mut expected = coalesce(&backlog);
    expected.push(calc_failed_seen());
    assert_seq("after the wait", &got, &expected);
    assert_end("after the wait", end, End::Empty);
    assert_eq!(
        h.non_input_drops(),
        0,
        "no control-event drop may be counted"
    );
}

#[test]
fn s7_service_failure_behind_backlog_without_reply_is_prompt() {
    run_bounded("s7_service_failure_no_reply", || {
        check_failure_behind_backlog_without_reply(Wait::Service)
    });
}

#[test]
fn s7_action_failure_behind_backlog_without_result_is_prompt() {
    run_bounded("s7_action_failure_no_reply", || {
        check_failure_behind_backlog_without_reply(Wait::Action)
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
        let outcome = Wait::Service.run(&mut h.events);
        Wait::Service.assert_reply(&outcome);

        let again: Vec<Change> = (0..BACKLOG)
            .step_by(3)
            .map(|i| set(key("p", i), 3_000_000 + i))
            .collect();
        h.send(events_of(&again));

        let (got, end) = read_ready(&mut h.events);
        let all: Vec<Change> = backlog.iter().chain(&again).cloned().collect();
        assert_seq("changes after the wait", &got, &coalesce(&all));
        assert_end("changes after the wait", end, End::Empty);
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
        let outcome = Wait::Service.run(&mut h.events);
        Wait::Service.assert_reply(&outcome);

        let (got, end) = read_ready(&mut h.events);
        let mut expected = coalesce(&backlog);
        expected.push(Seen::Stop(Cause::Manual));
        assert_seq("after the wait", &got, &expected);
        assert_end("after the wait", end, End::Closed);
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
        let outcome = Wait::Service.run(&mut h.events);
        Wait::Service.assert_reply(&outcome);

        let (got, end) = read_ready(&mut h.events);
        let mut expected = vec![Seen::Reload; NON_INPUT_EVENT_QUEUE_SIZE];
        expected.extend(coalesce(&backlog));
        assert_seq("after the wait", &got, &expected);
        assert_end("after the wait", end, End::Empty);
        assert_eq!(
            h.non_input_drops(),
            100,
            "only the Reload overflow may be counted"
        );
    });
}

/// Guard: a reply queued behind `Stop` is still found; `Stop` then reaches the
/// application, and the stream ends.
#[test]
fn guard_s7_reply_queued_behind_stop_is_found() {
    run_bounded("guard_s7_reply_behind_stop", || {
        let mut h = harness(64);
        h.send([Wait::Service.reply(), NodeEvent::Stop]);
        let outcome = Wait::Service.run(&mut h.events);
        Wait::Service.assert_reply(&outcome);
        let (got, end) = read_ready(&mut h.events);
        assert_seq("after the wait", &got, &[Seen::Stop(Cause::Manual)]);
        assert_end("after the wait", end, End::Closed);
    });
}

/// Guard: a failed server without a queued reply gives `StreamError` promptly.
#[test]
fn guard_s7_failed_server_without_reply_is_prompt_stream_error() {
    run_bounded("guard_s7_prompt_error", || {
        let mut h = harness(64);
        h.send([calc_failed()]);
        let outcome = Wait::Service.run_prompt(&mut h.events);
        assert_calc_failed(Wait::Service, &outcome);
        let (got, end) = read_ready(&mut h.events);
        assert_seq("after the wait", &got, &[calc_failed_seen()]);
        assert_end("after the wait", end, End::Empty);
    });
}

/// Guard: a wait after `Stop` without a reply gives `StreamEnded`.
#[test]
fn guard_s7_wait_after_stop_without_reply_is_stream_ended() {
    run_bounded("guard_s7_stream_ended", || {
        let mut h = harness(64);
        h.send([NodeEvent::Stop]);
        assert_eq!(h.events.recv().map(seen), Some(Seen::Stop(Cause::Manual)));
        let outcome = Wait::Service.run_prompt(&mut h.events);
        assert!(
            matches!(outcome, Err(PatternError::StreamEnded)),
            "expected StreamEnded, got {outcome:?}"
        );
        let (got, end) = read_ready(&mut h.events);
        assert_seq("after the wait", &got, &[]);
        assert_end("after the wait", end, End::Closed);
    });
}

/// Guard: `Stop` is never evicted from the events a wait passes over, and the
/// control-event overflow is counted.
#[test]
fn guard_s7_passthrough_never_evicts_stop() {
    run_bounded("guard_s7_stop_kept", || {
        let reloads = NON_INPUT_EVENT_QUEUE_SIZE + 200;
        let mut h = harness(reloads + 16);
        h.send((0..reloads).map(|_| reload()));
        h.send([NodeEvent::Stop, Wait::Service.reply()]);
        let outcome = Wait::Service.run(&mut h.events);
        Wait::Service.assert_reply(&outcome);

        let (got, end) = read_ready(&mut h.events);
        let mut expected = vec![Seen::Reload; NON_INPUT_EVENT_QUEUE_SIZE - 1];
        expected.push(Seen::Stop(Cause::Manual));
        assert_seq("after the wait", &got, &expected);
        assert_end("after the wait", end, End::Closed);
        assert_eq!(
            h.non_input_drops(),
            (reloads + 1 - NON_INPUT_EVENT_QUEUE_SIZE) as u64
        );
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
    let done = |got: &Vec<Seen>| got.last() == Some(&tick_seen());
    match path {
        ReadPath::Recv | ReadPath::RecvTimeout | ReadPath::RecvAsync => {
            for _ in 0..limit {
                let event = match path {
                    ReadPath::Recv => events.recv(),
                    ReadPath::RecvTimeout => events.recv_timeout(Duration::from_secs(5)),
                    _ => futures::executor::block_on(events.recv_async()),
                };
                got.push(seen(event.expect("stream open")));
                if done(&got) || matches!(got.last(), Some(Seen::Other(_))) {
                    break;
                }
            }
        }
        ReadPath::TryRecv => {
            let (all, end) = read_ready(events);
            assert_end("try_recv", end, End::Empty);
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

fn check_read_path(path: ReadPath) {
    let (sent, expected) = s8_scenario();
    let mut h = harness(sent.len() + 16);
    h.send(sent);
    let got = read_with(path, &mut h.events, expected.len() + 16);
    assert_seq(&format!("{path:?}"), &got, &expected);
    let (rest, end) = read_ready(&mut h.events);
    assert_seq(&format!("{path:?}: after the final input"), &rest, &[]);
    assert_end(&format!("{path:?}: after the final input"), end, End::Empty);
    assert_eq!(
        h.non_input_drops(),
        0,
        "no control-event drop may be counted"
    );
}

#[test]
fn s8_recv_gives_specified_parameter_sequence() {
    run_bounded("s8_recv", || check_read_path(ReadPath::Recv));
}

#[test]
fn s8_recv_timeout_gives_specified_parameter_sequence() {
    run_bounded("s8_recv_timeout", || check_read_path(ReadPath::RecvTimeout));
}

#[test]
fn s8_try_recv_gives_specified_parameter_sequence() {
    run_bounded("s8_try_recv", || check_read_path(ReadPath::TryRecv));
}

#[test]
fn s8_drain_gives_specified_parameter_sequence() {
    run_bounded("s8_drain", || check_read_path(ReadPath::Drain));
}

#[test]
fn s8_recv_async_gives_specified_parameter_sequence() {
    run_bounded("s8_recv_async", || check_read_path(ReadPath::RecvAsync));
}

/// Guard, S8: every read path gives the same sequence for the same pending
/// events.
#[test]
fn guard_s8_all_read_paths_agree() {
    run_bounded("guard_s8_agree", || {
        let paths = [
            ReadPath::Recv,
            ReadPath::RecvTimeout,
            ReadPath::TryRecv,
            ReadPath::Drain,
            ReadPath::RecvAsync,
        ];
        let mut results: Vec<(ReadPath, Vec<Seen>)> = Vec::new();
        for path in paths {
            let (sent, expected) = s8_scenario();
            let mut h = harness(sent.len() + 16);
            h.send(sent);
            results.push((path, read_with(path, &mut h.events, expected.len() + 16)));
        }
        let (first_path, first) = &results[0];
        for (path, got) in &results[1..] {
            assert_seq(&format!("{path:?} against {first_path:?}"), got, first);
        }
        // The sets of keys must agree too (a stricter message when they do not).
        let keys = |seq: &[Seen]| -> HashSet<String> {
            seq.iter()
                .filter_map(|s| s.param_key().map(str::to_string))
                .collect()
        };
        for (path, got) in &results[1..] {
            assert_eq!(keys(got), keys(first), "{path:?}: parameter keys differ");
        }
    });
}
