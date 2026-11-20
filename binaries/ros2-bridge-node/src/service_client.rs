//! The YAML service client: several requests in flight at once.

use std::{
    collections::BTreeMap,
    future::{Future, ready},
    pin::pin,
    sync::{Arc, Mutex, MutexGuard, PoisonError, Weak},
    task::Poll,
    time::Duration,
};

use arrow::array::{ArrayData, ArrayRef};
use dora_message::metadata::MetadataParameters;
use dora_node_api::Event;
use dora_ros2_bridge::ros2_client::{self, service::RmwRequestId};
use dora_ros2_bridge_arrow::{BridgeMessage, TypeInfo, TypeInfoGuard};
use futures::{
    FutureExt, Stream, StreamExt,
    channel::oneshot,
    future::{Either, select},
    stream::FuturesUnordered,
};

use crate::{DdsCallFailure, SERVICE_RESPONSE_TIMEOUT};

/// How many service requests a YAML service client keeps in flight; the same
/// bound as the YAML service server's pending requests.
pub(crate) const MAX_IN_FLIGHT_REQUESTS: usize = crate::MAX_PENDING_REQUESTS;

/// Call a service for each of `requests` and hand every outcome to
/// `respond`, with the parameters of the request it answers.
///
/// At most `limit` calls are in flight: while that many are, the next request
/// is not read. Outcomes come in completion order. Returns once `requests` has
/// ended and every call has finished, or with the first error of `respond`.
pub(crate) async fn drive_service_calls<P, D, T, F>(
    requests: impl Stream<Item = (P, D)>,
    limit: usize,
    mut call: impl FnMut(D) -> F,
    mut respond: impl FnMut(P, T) -> eyre::Result<()>,
) -> eyre::Result<()>
where
    F: Future<Output = T>,
{
    enum Step<R, O> {
        Request(R),
        Outcome(O),
        RequestsEnded,
        Done,
    }

    let mut requests = pin!(requests);
    let mut in_flight = FuturesUnordered::new();
    let mut requests_ended = false;
    loop {
        let step = futures::future::poll_fn(|cx| {
            if let Poll::Ready(Some(outcome)) = in_flight.poll_next_unpin(cx) {
                return Poll::Ready(Step::Outcome(outcome));
            }
            if !requests_ended && in_flight.len() < limit {
                match requests.as_mut().poll_next(cx) {
                    Poll::Ready(Some(request)) => return Poll::Ready(Step::Request(request)),
                    Poll::Ready(None) => return Poll::Ready(Step::RequestsEnded),
                    Poll::Pending => {}
                }
            }
            if requests_ended && in_flight.is_empty() {
                return Poll::Ready(Step::Done);
            }
            Poll::Pending
        })
        .await;
        match step {
            Step::Request((params, data)) => {
                in_flight.push(call(data).map(move |outcome| (params, outcome)));
            }
            Step::Outcome((params, outcome)) => respond(params, outcome)?,
            Step::RequestsEnded => requests_ended = true,
            Step::Done => return Ok(()),
        }
    }
}

/// The requests of a YAML service client: each input's parameters and data,
/// until the dataflow stops the node.
pub(crate) fn requests(
    events: impl Stream<Item = Event>,
) -> impl Stream<Item = (MetadataParameters, ArrayRef)> {
    events
        .take_while(|event| ready(!matches!(event, Event::Stop(_))))
        .filter_map(|event| {
            ready(match event {
                Event::Input { metadata, data, .. } => {
                    Some((metadata.parameters, data.as_array().clone()))
                }
                _ => None,
            })
        })
}

type DdsClient = ros2_client::Client<BridgeMessage, BridgeMessage>;

/// How often the response pump looks for new responses when there were none.
const RESPONSE_POLL_INTERVAL: Duration = Duration::from_millis(2);

/// The requests a DDS service client has sent and not had answered yet.
#[derive(Default)]
struct Waiting {
    senders: BTreeMap<RmwRequestId, oneshot::Sender<BridgeMessage>>,
    /// Why the response pump stopped, once it has: no response comes after.
    pump_error: Option<String>,
}

fn lock(waiting: &Mutex<Waiting>) -> MutexGuard<'_, Waiting> {
    waiting.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Concurrent calls on one DDS service client.
///
/// `Client::async_receive_response(id)` takes the responses for other ids
/// and drops them, so receivers for concurrent requests would steal each
/// other's responses (as in dora-rs/dora#1970). A single pump thread reads
/// every response instead and hands it to the request waiting for it.
pub(crate) struct DdsServiceCalls {
    client: Arc<DdsClient>,
    waiting: Arc<Mutex<Waiting>>,
    request_type_info: TypeInfo<'static>,
}

impl DdsServiceCalls {
    pub(crate) fn new(
        client: DdsClient,
        request_type_info: TypeInfo<'static>,
        response_type_info: TypeInfo<'static>,
    ) -> Self {
        let client = Arc::new(client);
        let waiting = Arc::new(Mutex::new(Waiting::default()));
        let pump = (client.clone(), Arc::downgrade(&waiting));
        std::thread::spawn(move || {
            let (client, waiting) = pump;
            let _guard = TypeInfoGuard::deserialize(response_type_info);
            pump_responses(&client, &waiting);
        });
        Self {
            client,
            waiting,
            request_type_info,
        }
    }

    /// Send a request and wait for its response, for at most
    /// [`SERVICE_RESPONSE_TIMEOUT`].
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
        async move {
            let id = sent?;
            let timeout = futures_timer::Delay::new(SERVICE_RESPONSE_TIMEOUT);
            match select(rx, timeout).await {
                Either::Left((Ok(response), _)) => Ok(response),
                Either::Left((Err(oneshot::Canceled), _)) => Err(DdsCallFailure::PumpStopped(
                    lock(&waiting).pump_error.clone().unwrap_or_default(),
                )),
                Either::Right(_) => {
                    lock(&waiting).senders.remove(&id);
                    Err(DdsCallFailure::TimedOut(SERVICE_RESPONSE_TIMEOUT))
                }
            }
        }
    }
}

/// Hand each response to the request waiting for it, until the
/// [`DdsServiceCalls`] is dropped or reading fails for good.
fn pump_responses(client: &DdsClient, waiting: &Weak<Mutex<Waiting>>) {
    while let Some(waiting) = waiting.upgrade() {
        match client.receive_response() {
            // A response nobody waits for timed out already, or answers
            // another client of the same service.
            Ok(Some((id, response))) => {
                if let Some(tx) = lock(&waiting).senders.remove(&id) {
                    let _ = tx.send(response);
                }
                continue;
            }
            Ok(None) => {}
            Err(error) => {
                let failure = DdsCallFailure::Read(error);
                if failure.is_fatal() {
                    let mut waiting = lock(&waiting);
                    waiting.pump_error = Some(failure.to_string());
                    // Fails the calls waiting now.
                    waiting.senders.clear();
                    return;
                }
                tracing::warn!("failed to receive ROS2 service response: {failure}");
            }
        }
        drop(waiting);
        std::thread::sleep(RESPONSE_POLL_INTERVAL);
    }
}

#[cfg(test)]
mod tests {
    use std::{cell::RefCell, rc::Rc};

    use futures::{channel::oneshot, executor::LocalPool, stream, task::LocalSpawnExt};

    use super::drive_service_calls;

    type Started = Rc<RefCell<Vec<(u32, oneshot::Sender<u32>)>>>;

    /// A driver run on a local pool, with calls the test completes by hand.
    struct Harness {
        pool: LocalPool,
        /// The data of each call started, with the sender that completes it.
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
                        |data| {
                            let (tx, rx) = oneshot::channel();
                            calls.borrow_mut().push((data, tx));
                            async move { rx.await.unwrap_or(0) }
                        },
                        |params, outcome| {
                            answers.borrow_mut().push((params, outcome));
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
    fn outcomes_come_in_completion_order_with_their_request_parameters() {
        let mut h = Harness::new(vec![("a", 1), ("b", 2), ("c", 3)], 8);
        h.run();
        h.complete(3, 30);
        h.complete(1, 10);
        h.complete(2, 20);
        assert_eq!(*h.responses.borrow(), vec![("c", 30), ("a", 10), ("b", 20)]);
    }

    #[test]
    fn no_more_than_limit_calls_are_in_flight() {
        let mut h = Harness::new(vec![("a", 1), ("b", 2), ("c", 3), ("d", 4)], 2);
        h.run();
        assert_eq!(h.started(), vec![1, 2]);
        h.complete(2, 20);
        assert_eq!(h.started(), vec![1, 3]);
        h.complete(1, 10);
        assert_eq!(h.started(), vec![3, 4]);
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
                    |data: u32| async move { data },
                    |_, outcome: u32| {
                        if outcome == 1 {
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
