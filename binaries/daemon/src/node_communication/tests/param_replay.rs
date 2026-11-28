use super::*;
use crate::coordinator::CoordinatorSender;
use crate::coordinator_events::{
    apply_state_catch_up_entries, deliver_param_delete_strict, deliver_param_update_strict,
};
use crate::tests::{daemon_and_events_rx, test_dataflow};
use crate::{Daemon, DaemonCoordinatorEvent, DaemonCoordinatorReply, RunningDataflow};
use dora_core::descriptor::DescriptorExt;
use dora_message::coordinator_to_daemon::{StateCatchUpEntry, StateCatchUpOperation};
use std::collections::BTreeMap;
use std::time::Duration;

async fn request(listener: &mut Listener, request: DaemonRequest) -> DaemonReply {
    let mut connection = RecordingReplies::default();
    tokio::time::timeout(
        Duration::from_secs(2),
        listener.handle_message(
            Timestamped {
                inner: request,
                timestamp: listener.clock.new_timestamp(),
            },
            &mut connection,
        ),
    )
    .await
    .expect("node request did not receive a reply")
    .unwrap();
    assert_eq!(connection.0.len(), 1);
    connection.0.pop().unwrap()
}

struct Subscriber {
    daemon: Daemon,
    events: mpsc::Receiver<Timestamped<Event>>,
    listener: Listener,
}

impl Subscriber {
    async fn new() -> Self {
        let (mut listener, _) = listener();
        let (sender, _sent) = CoordinatorSender::for_test();
        let (mut daemon, events) = daemon_and_events_rx(sender, listener.clock.clone()).await;
        daemon.coordinator_sender = None;
        let df = test_dataflow();
        daemon.running.insert(df.id, df);
        listener.daemon_tx = daemon.events_tx.clone();
        listener.subscribed_events = None;
        let mut fixture = Self {
            daemon,
            events,
            listener,
        };
        fixture.add_node().await;
        fixture
    }

    async fn add_node(&mut self) {
        // A dynamic node exercises actual local registration without starting a process.
        let descriptor: dora_message::descriptor::Descriptor =
            serde_json::from_value(serde_json::json!({
                "nodes": [{"id": self.listener.node_id, "path": "dynamic"}]
            }))
            .unwrap();
        let node = descriptor
            .resolve_aliases_and_set_defaults()
            .unwrap()
            .remove(&self.listener.node_id)
            .unwrap();
        let (tx, rx) = oneshot::channel();
        let _status = self
            .daemon
            .handle_coordinator_event(
                DaemonCoordinatorEvent::AddNode {
                    dataflow_id: self.listener.dataflow_id,
                    node,
                    uv: false,
                },
                tx,
            )
            .await
            .unwrap();
        let reply = rx.await.unwrap();
        assert!(
            matches!(reply, Some(DaemonCoordinatorReply::AddNodeResult(Ok(())))),
            "{reply:?}"
        );
    }

    fn dataflow(&mut self) -> &mut RunningDataflow {
        self.daemon
            .running
            .get_mut(&self.listener.dataflow_id)
            .unwrap()
    }

    async fn subscribe(&mut self) {
        let subscribe = request(&mut self.listener, DaemonRequest::Subscribe);
        let dispatch = async {
            let Event::Node {
                dataflow_id,
                node_id,
                event,
                ..
            } = self.events.recv().await.unwrap().inner
            else {
                panic!("expected Subscribe")
            };
            self.daemon
                .handle_node_event(event, dataflow_id, node_id)
                .await
                .unwrap();
        };
        let (reply, ()) = tokio::time::timeout(Duration::from_secs(2), async {
            tokio::join!(subscribe, dispatch)
        })
        .await
        .expect("Subscribe must complete before the node starts polling");
        assert!(matches!(reply, DaemonReply::Result(Ok(()))), "{reply:?}");
    }

    fn set(&mut self, key: &str, value: usize) -> eyre::Result<()> {
        let id = self.listener.node_id.clone();
        let clock = self.listener.clock.clone();
        deliver_param_update_strict(self.dataflow(), &id, key.into(), value.into(), &clock)
    }

    async fn updates_through_watermark(&mut self) -> BTreeMap<String, usize> {
        self.set("watermark", 7)
            .expect("fresh subscription should accept new writes");
        let mut values = BTreeMap::new();
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                // run_inner drains additional live events only after next_event
                // has accepted the first one. Bootstrap may defer that poll.
                if let Some(event) = futures::FutureExt::now_or_never(self.listener.next_event()) {
                    self.listener.note_drained();
                    self.listener.enqueue(event);
                    self.listener.handle_events().await.unwrap();
                }
                let DaemonReply::NextEvents(batch) =
                    request(&mut self.listener, DaemonRequest::NextEvent).await
                else {
                    panic!("expected a batch of events")
                };
                assert!(!batch.is_empty(), "listener closed before live watermark");
                for event in batch {
                    if let NodeEvent::ParamUpdate { key, value_json } = event.inner {
                        if key == "watermark" {
                            return;
                        }
                        values.insert(key, serde_json::from_slice(&value_json).unwrap());
                    }
                }
            }
        })
        .await
        .expect("live watermark must be delivered after replay");
        values
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn catch_up_after_first_failed_delivery_restores_latest_state() {
    for disconnected in [false, true] {
        let mut fixture = Subscriber::new().await;
        let id = fixture.listener.node_id.clone();
        let clock = fixture.listener.clock.clone();
        let (old, _old_events) = mpsc::channel(1);
        fixture
            .dataflow()
            .subscribe_channels
            .insert(id.clone(), old);
        fixture.set("y", 9).unwrap(); // fills the old subscriber's queue
        if disconnected {
            fixture.dataflow().mark_event_stream_dropped(&id);
        }
        let set_x = |value: usize| StateCatchUpOperation::SetParam {
            node_id: id.clone(),
            key: "x".into(),
            value: value.into(),
        };
        let entries = [
            StateCatchUpEntry {
                sequence: 1,
                operation: set_x(1),
            },
            StateCatchUpEntry {
                sequence: 2,
                operation: set_x(2),
            },
            StateCatchUpEntry {
                sequence: 3,
                operation: StateCatchUpOperation::DeleteParam {
                    node_id: id.clone(),
                    key: "y".into(),
                },
            },
        ];
        let ack = apply_state_catch_up_entries(fixture.dataflow(), &entries, &clock);
        assert_eq!(ack, 0, "no live delivery can be acknowledged");
        fixture.subscribe().await;
        assert_eq!(
            fixture.updates_through_watermark().await,
            BTreeMap::from([("x".into(), 2)])
        );
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn replay_larger_than_subscription_channel_precedes_newer_writes() {
    let mut fixture = Subscriber::new().await;
    let mut expected = BTreeMap::new();
    for value in 0..=NODE_EVENT_CHANNEL_CAPACITY {
        let key = format!("k{value:04}");
        assert!(fixture.set(&key, value).is_err(), "missing live subscriber");
        expected.insert(key, value);
    }
    let last = format!("k{NODE_EVENT_CHANNEL_CAPACITY:04}");
    fixture.subscribe().await;
    fixture.set(&last, 99).unwrap();
    expected.insert(last, 99);
    let observed = fixture.updates_through_watermark().await;
    assert_eq!(
        observed.len(),
        expected.len(),
        "every stored key must replay"
    );
    for (key, value) in expected {
        assert_eq!(observed.get(&key), Some(&value), "wrong value for {key}");
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn closed_subscriber_delete_is_reflected_in_replay() {
    let mut fixture = Subscriber::new().await;
    let id = fixture.listener.node_id.clone();
    let clock = fixture.listener.clock.clone();
    let (old, rx) = mpsc::channel(2);
    fixture
        .dataflow()
        .subscribe_channels
        .insert(id.clone(), old);
    fixture.set("keep", 1).unwrap();
    fixture.set("remove", 2).unwrap();
    drop(rx);
    assert!(fixture.set("keep", 3).is_err());
    assert!(deliver_param_delete_strict(fixture.dataflow(), &id, "remove".into(), &clock).is_err());
    fixture.subscribe().await;
    assert_eq!(
        fixture.updates_through_watermark().await,
        BTreeMap::from([("keep".into(), 3)])
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn subscribe_during_shutdown_receives_stop_before_params() {
    let mut fixture = Subscriber::new().await;
    assert!(fixture.set("gain", 1).is_err());
    fixture.dataflow().stop_sent = true;
    fixture.subscribe().await;
    fixture.listener.handle_events().await.unwrap();
    let DaemonReply::NextEvents(batch) =
        request(&mut fixture.listener, DaemonRequest::NextEvent).await
    else {
        panic!("expected a stop event")
    };
    assert!(matches!(
        batch.first().map(|e| &e.inner),
        Some(NodeEvent::Stop)
    ));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn remove_then_readd_does_not_replay_the_previous_nodes_parameters() {
    let mut fixture = Subscriber::new().await;
    let id = fixture.listener.node_id.clone();
    assert!(fixture.set("private-to-old-node", 1).is_err());
    let (tx, rx) = oneshot::channel();
    let _status = fixture
        .daemon
        .handle_coordinator_event(
            DaemonCoordinatorEvent::RemoveNode {
                dataflow_id: fixture.listener.dataflow_id,
                node_id: id.clone(),
                grace_duration: None,
            },
            tx,
        )
        .await
        .unwrap();
    assert!(matches!(
        rx.await.unwrap(),
        Some(DaemonCoordinatorReply::RemoveNodeResult(Ok(())))
    ));
    // A command already in flight can arrive after the old node was removed.
    assert!(fixture.set("late-direct", 2).is_err());
    let clock = fixture.listener.clock.clone();
    let delayed = StateCatchUpEntry {
        sequence: 1,
        operation: StateCatchUpOperation::SetParam {
            node_id: id.clone(),
            key: "late-catch-up".into(),
            value: 3.into(),
        },
    };
    assert_eq!(
        apply_state_catch_up_entries(fixture.dataflow(), &[delayed], &clock),
        0
    );
    fixture.add_node().await;
    fixture.subscribe().await;
    fixture.set("new-owner", 4).unwrap();
    fixture.dataflow().mark_event_stream_dropped(&id);
    let (mut successor, _) = listener();
    successor.daemon_tx = fixture.daemon.events_tx.clone();
    successor.subscribed_events = None;
    fixture.listener = successor;
    fixture.subscribe().await;
    assert_eq!(
        fixture.updates_through_watermark().await,
        BTreeMap::from([("new-owner".into(), 4)])
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn resubscribing_on_the_same_connection_preserves_unread_events() {
    let mut fixture = Subscriber::new().await;
    fixture.subscribe().await;
    fixture
        .daemon
        .send_reload(
            fixture.listener.dataflow_id,
            fixture.listener.node_id.clone(),
            None,
        )
        .await
        .unwrap();
    fixture.listener.handle_events().await.unwrap();
    // The node subscribed again before asking for its queued reload.
    fixture.subscribe().await;
    fixture.set("watermark", 7).unwrap();
    fixture.listener.handle_events().await.unwrap();
    let DaemonReply::NextEvents(batch) =
        request(&mut fixture.listener, DaemonRequest::NextEvent).await
    else {
        panic!("expected previously queued reload")
    };
    assert!(
        matches!(
            batch.first().map(|e| &e.inner),
            Some(NodeEvent::Reload { .. })
        ),
        "unread reload must precede the subsequent live update: {batch:?}"
    );
}
