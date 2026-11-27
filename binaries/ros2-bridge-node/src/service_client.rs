//! The YAML service client: several requests in flight at once.

use std::{
    cell::{Cell, RefCell},
    collections::{BTreeMap, VecDeque},
    future::{Future, ready},
    panic::AssertUnwindSafe,
    pin::pin,
    rc::Rc,
    sync::{Arc, Mutex, MutexGuard, PoisonError, Weak},
    task::Poll,
    time::{Duration, Instant},
};

use arrow::array::{ArrayData, ArrayRef};
use dora_message::metadata::MetadataParameters;
use dora_node_api::Event;
use dora_ros2_bridge::{
    ros2_client::{self, service::RmwRequestId},
    transport::History,
};
use dora_ros2_bridge_arrow::{BridgeMessage, TypeInfo, TypeInfoGuard};
use futures::{
    FutureExt, Stream, StreamExt,
    channel::oneshot,
    future::{self, AbortHandle, Abortable},
};
use futures_timer::Delay;

use eyre::Context;

use crate::DdsCallFailure;

/// The most service requests a YAML service client keeps in flight; the same
/// bound as the YAML service server's pending requests.
pub(crate) const MAX_IN_FLIGHT_REQUESTS: usize = crate::MAX_PENDING_REQUESTS;

/// How many requests a YAML service client keeps in flight, for the history
/// of its service QoS: its `keep_last` depth, at most
/// [`MAX_IN_FLIGHT_REQUESTS`]. A KeepLast history holds that many samples, so
/// more requests at once could push older requests out of a server's request
/// history, or responses out of the bridge's own, before they are read.
pub(crate) fn in_flight_limit(history: History) -> usize {
    match history {
        History::KeepAll => MAX_IN_FLIGHT_REQUESTS,
        History::KeepLast { depth } => usize::try_from(depth)
            .unwrap_or(0)
            .clamp(1, MAX_IN_FLIGHT_REQUESTS),
    }
}

/// Call a service for each of `requests` and hand every outcome to
/// `respond`, with the parameters of the request it answers: what the call
/// returned, or `None` when the server stopped answering and the call was
/// given up.
///
/// The oldest unanswered call is given up once `timeout` has passed both
/// since it was sent and since the last answer to any call. A server that
/// answers one request at a time keeps answering, so its queued requests never
/// time out; a server that stops answering fails all of them about `timeout`
/// later, and a single dropped request fails `timeout` after the answers stop.
///
/// At most `limit` calls are in flight: while that many are, the next request
/// is not read. Outcomes come in request order, as with one call at a time, so
/// a dataflow that pairs responses with requests by position keeps working; a
/// call that finished early waits, holding its slot, for the calls before it.
/// Returns once `requests` has ended and every call has finished, or with the
/// first error of `respond`.
pub(crate) async fn drive_service_calls<P, D, T, F>(
    requests: impl Stream<Item = (P, D)>,
    limit: usize,
    timeout: Duration,
    mut call: impl FnMut(D) -> F,
    mut respond: impl FnMut(P, Option<T>) -> eyre::Result<()>,
) -> eyre::Result<()>
where
    F: Future<Output = T>,
{
    /// A call not yet handed to `respond`, in request order.
    struct Sent {
        at: Instant,
        give_up: AbortHandle,
    }
    let sent: Rc<RefCell<VecDeque<Sent>>> = Rc::default();
    // When the last answer to any call arrived.
    let last_answer: Rc<Cell<Option<Instant>>> = Rc::default();

    let outcomes = requests
        .map(|(params, data)| {
            let (give_up, registration) = AbortHandle::new_pair();
            sent.borrow_mut().push_back(Sent {
                at: Instant::now(),
                give_up,
            });
            let last_answer = last_answer.clone();
            Abortable::new(call(data), registration).map(move |outcome| {
                if outcome.is_ok() {
                    last_answer.set(Some(Instant::now()));
                }
                (params, outcome.ok())
            })
        })
        .buffered(limit);
    let mut outcomes = pin!(outcomes);

    // When the oldest unanswered call is due to be given up.
    let deadline = || {
        sent.borrow().front().map(|oldest| {
            let since = last_answer
                .get()
                .map_or(oldest.at, |answer| answer.max(oldest.at));
            since + timeout
        })
    };
    enum Step<O> {
        Outcome(Option<O>),
        Due,
    }
    // The timer for the current deadline, rebuilt when the deadline moves.
    let mut timer: Option<(Instant, Delay)> = None;
    loop {
        let step = future::poll_fn(|cx| {
            if let Poll::Ready(outcome) = outcomes.as_mut().poll_next(cx) {
                return Poll::Ready(Step::Outcome(outcome));
            }
            // Polling the outcomes sends new calls and records answers, so
            // the deadline is computed after it.
            let Some(due) = deadline() else {
                timer = None;
                return Poll::Pending;
            };
            let delay = match &mut timer {
                Some((at, delay)) if *at == due => delay,
                _ => {
                    let delay = Delay::new(due.saturating_duration_since(Instant::now()));
                    &mut timer.insert((due, delay)).1
                }
            };
            delay.poll_unpin(cx).map(|()| Step::Due)
        })
        .await;
        match step {
            Step::Outcome(Some((params, outcome))) => {
                sent.borrow_mut().pop_front();
                respond(params, outcome)?;
            }
            Step::Outcome(None) => return Ok(()),
            Step::Due => {
                timer = None;
                if let Some(oldest) = sent.borrow().front() {
                    oldest.give_up.abort();
                }
            }
        }
    }
}

/// The parameters a response carries: those of its request, minus the keys
/// that steer how dora delivers a message rather than describe it.
///
/// Copying `flush`, for one, would make the requester's input queue drop the
/// responses waiting behind this one. Keys starting with `_` are dora's own.
fn response_parameters(mut request: MetadataParameters) -> MetadataParameters {
    use dora_message::metadata::{FIN, FLUSH, OPEN_TELEMETRY_CONTEXT, SEGMENT_ID, SEQ, SESSION_ID};
    request.retain(|key, _| {
        !key.starts_with('_')
            && ![
                FLUSH,
                FIN,
                SEQ,
                SESSION_ID,
                SEGMENT_ID,
                OPEN_TELEMETRY_CONTEXT,
            ]
            .contains(&key.as_str())
    });
    request
}

/// The requests of a YAML service client: each input's data and the
/// parameters its response will carry, until the dataflow stops the node.
pub(crate) fn requests(
    events: impl Stream<Item = Event>,
) -> impl Stream<Item = (MetadataParameters, ArrayRef)> {
    events
        .take_while(|event| ready(!matches!(event, Event::Stop(_))))
        .filter_map(|event| {
            ready(match event {
                Event::Input { metadata, data, .. } => Some((
                    response_parameters(metadata.parameters),
                    data.as_array().clone(),
                )),
                _ => None,
            })
        })
}

type DdsClient = ros2_client::Client<BridgeMessage, BridgeMessage>;

/// The pump's `mio` tokens: responses to read, or the calls were dropped.
const RESPONSES: mio::Token = mio::Token(0);
const DROPPED: mio::Token = mio::Token(1);

/// The requests a DDS service client has sent and not had answered yet.
#[derive(Default)]
struct Waiting {
    senders: BTreeMap<RmwRequestId, oneshot::Sender<BridgeMessage>>,
    /// Why the response pump stopped, once it has: no response comes after.
    pump_error: Option<String>,
}

impl Waiting {
    /// Hand `response` to the request `id`; `false` if none waits for it.
    fn deliver(&mut self, id: RmwRequestId, response: BridgeMessage) -> bool {
        match self.senders.remove(&id) {
            Some(tx) => {
                let _ = tx.send(response);
                true
            }
            None => false,
        }
    }
}

/// Forgets a request when its call is given up before the response comes.
struct Registered {
    waiting: Arc<Mutex<Waiting>>,
    id: RmwRequestId,
}

impl Drop for Registered {
    fn drop(&mut self) {
        lock(&self.waiting).senders.remove(&self.id);
    }
}

/// Wait for the response to request `id`. The caller decides when to give up
/// by dropping the future, which forgets the request.
async fn await_response(
    waiting: Arc<Mutex<Waiting>>,
    id: RmwRequestId,
    response: oneshot::Receiver<BridgeMessage>,
) -> Result<BridgeMessage, DdsCallFailure> {
    let registered = Registered { waiting, id };
    response.await.map_err(|oneshot::Canceled| {
        DdsCallFailure::PumpStopped(
            lock(&registered.waiting)
                .pump_error
                .clone()
                .unwrap_or_else(|| "the response thread stopped".to_owned()),
        )
    })
}

fn lock(waiting: &Mutex<Waiting>) -> MutexGuard<'_, Waiting> {
    waiting.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Record why the response pump stopped, and fail the calls waiting now.
fn pump_stopped(waiting: &Mutex<Waiting>, error: String) {
    let mut waiting = lock(waiting);
    waiting.pump_error = Some(error);
    waiting.senders.clear();
}

/// Concurrent calls on one DDS service client.
///
/// `Client::async_receive_response(id)` takes the responses for other ids
/// and drops them, so receivers for concurrent requests would steal each
/// other's responses (as in dora-rs/dora#1970). A single pump thread reads
/// every response instead and hands it to the request waiting for it. It
/// sleeps in `mio` until the client's response reader has data.
pub(crate) struct DdsServiceCalls {
    client: Arc<DdsClient>,
    waiting: Arc<Mutex<Waiting>>,
    request_type_info: TypeInfo<'static>,
    /// Ends the pump when the calls are dropped.
    stop_pump: mio::Waker,
}

impl Drop for DdsServiceCalls {
    fn drop(&mut self) {
        let _ = self.stop_pump.wake();
    }
}

impl DdsServiceCalls {
    pub(crate) fn new(
        mut client: DdsClient,
        request_type_info: TypeInfo<'static>,
        response_type_info: TypeInfo<'static>,
    ) -> eyre::Result<Self> {
        let poll = mio::Poll::new().context("failed to create the service response poll")?;
        poll.registry()
            .register(&mut client, RESPONSES, mio::Interest::READABLE)
            .context("failed to watch the service response reader")?;
        let stop_pump = mio::Waker::new(poll.registry(), DROPPED)
            .context("failed to create the service response waker")?;
        let client = Arc::new(client);
        let waiting = Arc::new(Mutex::new(Waiting::default()));
        let pump = (client.clone(), Arc::downgrade(&waiting));
        std::thread::spawn(move || {
            let (client, waiting) = pump;
            let _guard = TypeInfoGuard::deserialize(response_type_info);
            // A panic while decoding a response must not leave every call
            // waiting for its timeout: fail them, as a fatal read error does.
            let pumped = std::panic::catch_unwind(AssertUnwindSafe(|| {
                pump_responses(&client, &waiting, poll)
            }));
            if pumped.is_err()
                && let Some(waiting) = waiting.upgrade()
            {
                pump_stopped(&waiting, "the response thread panicked".to_owned());
            }
        });
        Ok(Self {
            client,
            waiting,
            request_type_info,
            stop_pump,
        })
    }

    /// Send a request and wait for its response, until the future is dropped.
    pub(crate) fn call(
        &self,
        data: ArrayData,
    ) -> impl Future<Output = Result<BridgeMessage, DdsCallFailure>> + use<> {
        let (tx, rx) = oneshot::channel();
        // Registered under the lock before the pump can look the id up, so a
        // response that arrives at once still finds its request.
        let sent = {
            let mut waiting = lock(&self.waiting);
            match &waiting.pump_error {
                Some(error) => Err(DdsCallFailure::PumpStopped(error.clone())),
                None => {
                    let _guard = TypeInfoGuard::serialize(self.request_type_info.clone());
                    self.client
                        .send_request(BridgeMessage(Some(data)))
                        .map_err(DdsCallFailure::Write)
                        .inspect(|&id| {
                            waiting.senders.insert(id, tx);
                        })
                }
            }
        };
        let waiting = self.waiting.clone();
        async move { await_response(waiting, sent?, rx).await }
    }
}

/// Hand each response to the request waiting for it, until the
/// [`DdsServiceCalls`] is dropped or reading fails for good.
fn pump_responses(client: &DdsClient, calls: &Weak<Mutex<Waiting>>, mut poll: mio::Poll) {
    let mut events = mio::Events::with_capacity(4);
    loop {
        let Some(waiting) = calls.upgrade() else {
            return;
        };
        // Everything that has arrived; `receive_response` also resets the
        // reader's readiness, so data that comes after it wakes the poll.
        loop {
            match client.receive_response() {
                // A response nobody waits for timed out already, or answers
                // another client of the same service.
                Ok(Some((id, response))) => {
                    lock(&waiting).deliver(id, response);
                }
                Ok(None) => break,
                Err(error) => {
                    let failure = DdsCallFailure::Read(error);
                    if failure.is_fatal() {
                        pump_stopped(&waiting, failure.to_string());
                        return;
                    }
                    // One undecodable response: the next may be waiting already.
                    tracing::warn!("failed to receive ROS2 service response: {failure}");
                }
            }
        }
        drop(waiting);
        match poll.poll(&mut events, None) {
            Ok(()) => {}
            Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(error) => {
                if let Some(waiting) = calls.upgrade() {
                    pump_stopped(&waiting, format!("waiting for responses failed: {error}"));
                }
                return;
            }
        }
        if events.iter().any(|event| event.token() == DROPPED) {
            return;
        }
    }
}

#[cfg(test)]
mod timeout_tests {
    use std::{
        future::Future,
        time::{Duration, Instant},
    };

    use futures::{
        executor::block_on,
        future::{self, Either, select},
        stream,
    };
    use futures_timer::Delay;

    use super::drive_service_calls;

    const TIMEOUT: Duration = Duration::from_millis(200);

    /// How a fake server answers call `n` (from 1): after this long, or never.
    type Answer = Option<Duration>;

    /// Run calls answered as `answers` says; the outcomes in order and the
    /// time the driver took. Fails if it has not finished after 2 s.
    fn run(answers: Vec<Answer>) -> (Vec<(usize, Option<usize>)>, Duration) {
        let start = Instant::now();
        let requests = stream::iter(
            answers
                .into_iter()
                .enumerate()
                .map(|(i, a)| (i + 1, (i + 1, a))),
        );
        let mut outcomes = Vec::new();
        let driver = drive_service_calls(
            requests,
            8,
            TIMEOUT,
            |(n, answer): (usize, Answer)| -> std::pin::Pin<Box<dyn Future<Output = usize>>> {
                match answer {
                    Some(after) => Box::pin(async move {
                        Delay::new(after).await;
                        n
                    }),
                    None => Box::pin(future::pending()),
                }
            },
            |n, outcome| {
                outcomes.push((n, outcome));
                Ok(())
            },
        );
        let guard = Delay::new(Duration::from_secs(2));
        match block_on(select(Box::pin(driver), guard)) {
            Either::Left((result, _)) => result.expect("driver"),
            Either::Right(_) => panic!("the driver did not finish within 2 s"),
        }
        (outcomes, start.elapsed())
    }

    #[test]
    fn a_server_that_stops_answering_fails_every_call_after_one_timeout() {
        let (outcomes, took) = run(vec![None, None, None]);
        assert_eq!(outcomes, vec![(1, None), (2, None), (3, None)]);
        // Not one timeout per call ahead: 3 x 200 ms would be 600 ms. The
        // bound sits right below that, so a slow CI timer does not trip it.
        assert!(took >= TIMEOUT && took < TIMEOUT * 3, "took {took:?}");
    }

    #[test]
    fn a_server_answering_one_at_a_time_is_never_cut_off() {
        // Answers 150 ms apart: the last one 600 ms after its send, well past
        // the timeout, but never 200 ms after the previous answer.
        let answers = (1..=4)
            .map(|n| Some(Duration::from_millis(150 * n)))
            .collect();
        let (outcomes, _) = run(answers);
        assert_eq!(
            outcomes,
            vec![(1, Some(1)), (2, Some(2)), (3, Some(3)), (4, Some(4))]
        );
    }

    #[test]
    fn a_dropped_request_fails_one_timeout_after_the_answers_stop() {
        const LAST_ANSWER: Duration = Duration::from_millis(150);
        let answers = vec![Some(Duration::from_millis(20)), None, Some(LAST_ANSWER)];
        let (outcomes, took) = run(answers);
        // Call 3, answered early, still comes out after call 2.
        assert_eq!(outcomes, vec![(1, Some(1)), (2, None), (3, Some(3))]);
        // Call 2 is given up 200 ms after the last answer (150 ms), not 200 ms
        // after its own send. Only the lower bound tells those apart; an upper
        // bound would only measure the runner's timer latency (#4331), and the
        // 2 s guard in `run` already catches a driver that never gives up.
        assert!(took >= LAST_ANSWER + TIMEOUT, "took {took:?}");
    }
}

#[cfg(test)]
mod waiting_tests {
    use std::sync::{Arc, Mutex};

    use dora_ros2_bridge::ros2_client::service::RmwRequestId;
    use dora_ros2_bridge_arrow::BridgeMessage;
    use futures::{FutureExt, channel::oneshot, executor::block_on};

    use super::{Waiting, await_response, lock, pump_stopped};
    use crate::DdsCallFailure;

    fn id(sequence_number: i64) -> RmwRequestId {
        RmwRequestId {
            sequence_number,
            ..Default::default()
        }
    }

    /// Register request `n` as the client does, returning its receiver.
    fn register(waiting: &Mutex<Waiting>, n: i64) -> oneshot::Receiver<BridgeMessage> {
        let (tx, rx) = oneshot::channel();
        lock(waiting).senders.insert(id(n), tx);
        rx
    }

    #[test]
    fn a_response_reaches_the_request_with_its_id() {
        let waiting = Arc::new(Mutex::new(Waiting::default()));
        let first = register(&waiting, 1);
        let second = register(&waiting, 2);
        assert!(lock(&waiting).deliver(id(2), BridgeMessage(None)));
        assert!(block_on(await_response(waiting.clone(), id(2), second)).is_ok());
        // The other request is still waiting for its own response.
        assert_eq!(lock(&waiting).senders.len(), 1);
        assert!(lock(&waiting).deliver(id(1), BridgeMessage(None)));
        assert!(block_on(await_response(waiting.clone(), id(1), first)).is_ok());
    }

    #[test]
    fn a_response_nobody_waits_for_is_dropped() {
        let waiting = Arc::new(Mutex::new(Waiting::default()));
        let _first = register(&waiting, 1);
        assert!(!lock(&waiting).deliver(id(7), BridgeMessage(None)));
        assert_eq!(lock(&waiting).senders.len(), 1);
    }

    #[test]
    fn a_request_given_up_is_forgotten() {
        let waiting = Arc::new(Mutex::new(Waiting::default()));
        let rx = register(&waiting, 1);
        let mut call = Box::pin(await_response(waiting.clone(), id(1), rx));
        assert!((&mut call).now_or_never().is_none());
        // The driver gives up on the call by dropping it.
        drop(call);
        assert!(lock(&waiting).senders.is_empty());
        // Its late response finds nobody.
        assert!(!lock(&waiting).deliver(id(1), BridgeMessage(None)));
    }

    #[test]
    fn a_stopped_pump_fails_the_waiting_requests_with_its_error() {
        let waiting = Arc::new(Mutex::new(Waiting::default()));
        let rx = register(&waiting, 1);
        pump_stopped(&waiting, "rtps thread panicked".to_owned());
        let outcome = block_on(await_response(waiting.clone(), id(1), rx));
        match outcome {
            Err(DdsCallFailure::PumpStopped(error)) => assert_eq!(error, "rtps thread panicked"),
            other => panic!("expected PumpStopped, got {other:?}"),
        }
        assert!(lock(&waiting).pump_error.is_some());
    }
}

#[cfg(test)]
mod response_parameters_tests {
    use dora_message::metadata::{
        FIN, FLUSH, FRAMING, MetadataParameters, OPEN_TELEMETRY_CONTEXT, Parameter, REQUEST_ID,
        SEGMENT_ID, SEQ, SESSION_ID,
    };

    use super::response_parameters;

    #[test]
    fn a_response_keeps_the_request_ids_and_drops_delivery_keys() {
        let mut request = MetadataParameters::default();
        request.insert("request_number".into(), Parameter::Integer(7));
        request.insert(REQUEST_ID.into(), Parameter::String("r".into()));
        for key in [FLUSH, FIN] {
            request.insert(key.into(), Parameter::Bool(true));
        }
        for key in [
            SESSION_ID,
            SEGMENT_ID,
            OPEN_TELEMETRY_CONTEXT,
            FRAMING,
            "__dora_x",
        ] {
            request.insert(key.into(), Parameter::String("x".into()));
        }
        request.insert(SEQ.into(), Parameter::Integer(3));

        let response = response_parameters(request);
        let keys: Vec<&str> = response.keys().map(String::as_str).collect();
        assert_eq!(keys, vec![REQUEST_ID, "request_number"]);
    }
}

#[cfg(test)]
mod in_flight_limit_tests {
    use dora_ros2_bridge::transport::History;

    use super::{MAX_IN_FLIGHT_REQUESTS, in_flight_limit};

    #[test]
    fn the_limit_is_the_service_history_depth() {
        assert_eq!(in_flight_limit(History::KeepLast { depth: 10 }), 10);
        assert_eq!(in_flight_limit(History::KeepLast { depth: 3 }), 3);
    }

    #[test]
    fn the_limit_is_capped_and_never_zero() {
        assert_eq!(
            in_flight_limit(History::KeepLast { depth: 1000 }),
            MAX_IN_FLIGHT_REQUESTS
        );
        assert_eq!(in_flight_limit(History::KeepAll), MAX_IN_FLIGHT_REQUESTS);
        assert_eq!(in_flight_limit(History::KeepLast { depth: 0 }), 1);
        assert_eq!(in_flight_limit(History::KeepLast { depth: -5 }), 1);
    }
}

#[cfg(test)]
mod tests {
    use std::{cell::RefCell, rc::Rc, time::Duration};

    use futures::{channel::oneshot, executor::LocalPool, stream, task::LocalSpawnExt};

    use super::drive_service_calls;

    /// Each started call: its data and the sender that completes it.
    type Started = Rc<RefCell<Vec<(u32, oneshot::Sender<u32>)>>>;

    /// Long enough that no call in these tests is given up.
    const TIMEOUT: Duration = Duration::from_secs(3600);

    /// A driver run on a local pool, with calls the test completes by hand.
    struct Harness {
        pool: LocalPool,
        /// The calls started and not completed yet.
        started: Started,
        /// `(parameters, outcome)` of each finished call, in order.
        responses: Rc<RefCell<Vec<(&'static str, u32)>>>,
        /// The driver's result, once it has returned.
        result: Rc<RefCell<Option<eyre::Result<()>>>>,
    }

    impl Harness {
        fn new(requests: Vec<(&'static str, u32)>, limit: usize) -> Self {
            let pool = LocalPool::new();
            let started = Rc::new(RefCell::new(Vec::new()));
            let responses = Rc::new(RefCell::new(Vec::new()));
            let result = Rc::new(RefCell::new(None));
            let (calls, answers, done) = (started.clone(), responses.clone(), result.clone());
            pool.spawner()
                .spawn_local(async move {
                    let outcome = drive_service_calls(
                        stream::iter(requests),
                        limit,
                        TIMEOUT,
                        |data| {
                            let (tx, rx) = oneshot::channel();
                            calls.borrow_mut().push((data, tx));
                            async move { rx.await.unwrap_or(0) }
                        },
                        |params, outcome| {
                            answers
                                .borrow_mut()
                                .push((params, outcome.expect("answered")));
                            Ok(())
                        },
                    )
                    .await;
                    *done.borrow_mut() = Some(outcome);
                })
                .expect("spawn the driver");
            Self {
                pool,
                started,
                responses,
                result,
            }
        }

        fn run(&mut self) {
            self.pool.run_until_stalled();
        }

        fn started(&self) -> Vec<u32> {
            self.started
                .borrow()
                .iter()
                .map(|(data, _)| *data)
                .collect()
        }

        /// Complete the started call for `data` with `outcome`.
        fn complete(&mut self, data: u32, outcome: u32) {
            let position = self
                .started
                .borrow()
                .iter()
                .position(|(started, _)| *started == data)
                .expect("call was started");
            let (_, tx) = self.started.borrow_mut().remove(position);
            let _ = tx.send(outcome);
            self.run();
        }

        fn finished(&self) -> bool {
            self.result.borrow().is_some()
        }
    }

    #[test]
    fn several_calls_are_in_flight_at_once() {
        let mut h = Harness::new(vec![("a", 1), ("b", 2), ("c", 3)], 8);
        h.run();
        assert_eq!(h.started(), vec![1, 2, 3]);
        assert!(h.responses.borrow().is_empty());
    }

    #[test]
    fn outcomes_come_in_request_order_with_their_request_parameters() {
        let mut h = Harness::new(vec![("a", 1), ("b", 2), ("c", 3)], 8);
        h.run();
        // Answered out of order: each waits for the requests before it.
        h.complete(3, 30);
        assert!(h.responses.borrow().is_empty());
        h.complete(1, 10);
        assert_eq!(*h.responses.borrow(), vec![("a", 10)]);
        h.complete(2, 20);
        assert_eq!(*h.responses.borrow(), vec![("a", 10), ("b", 20), ("c", 30)]);
    }

    #[test]
    fn no_more_than_limit_calls_are_in_flight() {
        let mut h = Harness::new(vec![("a", 1), ("b", 2), ("c", 3), ("d", 4)], 2);
        h.run();
        assert_eq!(h.started(), vec![1, 2]);
        h.complete(1, 10);
        assert_eq!(h.started(), vec![2, 3]);
        // A finished call keeps its slot until the calls before it are out.
        h.complete(3, 30);
        assert_eq!(h.started(), vec![2]);
        h.complete(2, 20);
        assert_eq!(h.started(), vec![4]);
    }

    #[test]
    fn calls_in_flight_finish_after_the_requests_end() {
        let mut h = Harness::new(vec![("a", 1), ("b", 2)], 8);
        h.run();
        // The request stream has ended; the driver still waits for both.
        assert!(!h.finished());
        h.complete(1, 10);
        assert!(!h.finished());
        h.complete(2, 20);
        assert!(h.finished());
        assert!(h.result.borrow().as_ref().is_some_and(|r| r.is_ok()));
        assert_eq!(*h.responses.borrow(), vec![("a", 10), ("b", 20)]);
    }

    #[test]
    fn an_error_from_respond_ends_the_driver() {
        let mut pool = LocalPool::new();
        let result = Rc::new(RefCell::new(None));
        let done = result.clone();
        pool.spawner()
            .spawn_local(async move {
                let outcome = drive_service_calls(
                    stream::iter(vec![((), 1), ((), 2)]),
                    8,
                    TIMEOUT,
                    |data: u32| async move { data },
                    |_, outcome: Option<u32>| {
                        if outcome == Some(1) {
                            eyre::bail!("fatal")
                        }
                        Ok(())
                    },
                )
                .await;
                *done.borrow_mut() = Some(outcome);
            })
            .expect("spawn the driver");
        pool.run_until_stalled();
        let result = result.borrow();
        let error = result
            .as_ref()
            .expect("driver returned")
            .as_ref()
            .unwrap_err();
        assert_eq!(error.to_string(), "fatal");
    }
}
