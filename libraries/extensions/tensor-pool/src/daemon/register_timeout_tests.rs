//! The origin side of a cross-machine register that never gets its ack.

use super::test_services::{FakeServices, enable_cross_machine, linked_sessions};
use super::*;

/// An origin that gives up on a register after the last ack timeout
/// must tell the mirror to drop whatever it created: the mirror may
/// have built the segment while only its ack was lost, and no later
/// free from the node would reach it (#4101).
// The guard only serializes the env var across tests; holding it over
// the awaits is the point.
#[allow(clippy::await_holding_lock)]
#[tokio::test(flavor = "multi_thread", worker_threads = 1)]
async fn timed_out_register_frees_the_mirror() {
    let _env = enable_cross_machine();
    let (mirror_session, origin_session) = linked_sessions().await;
    let (mut origin, _sink) = FakeServices::with_session("A", origin_session);
    origin.resolved = Some("127.0.0.1:1".parse().unwrap());
    let df = Uuid::new_v4();
    let pool = "pool_sender_1";

    // The mirror: records every pool message and never acks, as if
    // each ack were lost on the way back.
    let mirror = mirror_session
        .declare_subscriber(pool_message_topic(&df, None))
        .await
        .unwrap();
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    let mut state = PoolState::new();
    let (reply_tx, reply_rx) = oneshot::channel();
    let request = NodeRequest::RegisterCrossMachine {
        shared_memory_id: pool.to_string(),
        shmem_name: "dora_test_sender_segment".to_string(),
        size: 4096,
        dtype: "uint8".to_string(),
        shape: vec![4096],
        device: "cpu".to_string(),
        machine_id: "B".to_string(),
    };
    state
        .handle_extension_request(
            &mut origin,
            df,
            "sender".parse().unwrap(),
            NAMESPACE.to_string(),
            postcard::to_allocvec(&request).unwrap(),
            reply_tx,
        )
        .await
        .unwrap();

    let reply = tokio::time::timeout(CROSS_REGISTER_TIMEOUT * 4, reply_rx)
        .await
        .expect("the register must give up after its retries")
        .unwrap();
    let DaemonReply::ExtensionReply { payload } = reply else {
        panic!("unexpected reply: {reply:?}")
    };
    let response = postcard::from_bytes(&payload).unwrap();
    let NodeResponse::CrossMachineRegistered { result, .. } = response else {
        panic!("unexpected response: {response:?}")
    };
    assert!(
        result.unwrap_err().contains("timed out"),
        "the register must fail on the ack timeout"
    );

    let mut registers = 0;
    let mut freed = false;
    while let Ok(Ok(sample)) =
        tokio::time::timeout(std::time::Duration::from_secs(2), mirror.recv_async()).await
    {
        let event = Timestamped::<InterDaemonEvent>::deserialize_inter_daemon_event(
            &sample.payload().to_bytes(),
        )
        .unwrap();
        let InterDaemonEvent::ExtensionMessage {
            target_machine,
            payload,
            ..
        } = event.inner
        else {
            panic!("unexpected event: {:?}", event.inner)
        };
        assert_eq!(target_machine.as_deref(), Some("B"));
        match postcard::from_bytes(&payload).unwrap() {
            PeerMessage::Register { .. } => {
                assert!(!freed, "no register may follow the rollback free");
                registers += 1;
            }
            PeerMessage::Free { shared_memory_id } => {
                assert_eq!(shared_memory_id, pool);
                freed = true;
            }
            other => panic!("unexpected pool message: {other:?}"),
        }
    }
    assert_eq!(registers, 3, "the origin retries the register twice");
    assert!(freed, "the origin must free the mirror after giving up");
}
