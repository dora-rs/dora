//! End-to-end tests: coordinator + CLI WsSession over real WebSocket connections.
//!
//! `WsSession` creates its own tokio runtime internally (`block_on`), so these
//! tests run the coordinator on a background thread and use `WsSession` from
//! the main test thread (no nested runtimes).

use adora_cli::WsSession;
use adora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use std::net::SocketAddr;

/// Start a coordinator on a background tokio runtime. Returns the bound port.
fn start_coordinator_background() -> u16 {
    let (port_tx, port_rx) = std::sync::mpsc::channel();

    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("failed to create coordinator runtime");

        rt.block_on(async {
            let bind: SocketAddr = "127.0.0.1:0".parse().unwrap();
            let (port, future) = adora_coordinator::start_testing(bind, futures::stream::empty())
                .await
                .expect("failed to start coordinator");
            port_tx.send(port).unwrap();
            let _ = future.await;
        });
    });

    let port = port_rx.recv().expect("failed to receive coordinator port");
    // Poll until the coordinator is accepting connections (up to 2s)
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        if std::net::TcpStream::connect(format!("127.0.0.1:{port}")).is_ok() {
            break;
        }
        if std::time::Instant::now() > deadline {
            panic!("coordinator did not become ready within 2s");
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    port
}

/// Helper: send a ControlRequest via WsSession and deserialize the reply.
fn send_request(session: &WsSession, req: &ControlRequest) -> eyre::Result<ControlRequestReply> {
    let data = serde_json::to_vec(req)?;
    let reply_bytes = session.request(&data)?;
    let reply: ControlRequestReply = serde_json::from_slice(&reply_bytes)?;
    Ok(reply)
}

#[test]
fn cli_list_empty() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(&session, &ControlRequest::List).unwrap();
    match reply {
        ControlRequestReply::DataflowList(list) => {
            assert!(list.0.is_empty(), "expected empty dataflow list");
        }
        other => panic!("expected DataflowList, got {other:?}"),
    }
}

#[test]
fn cli_status_no_daemon() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let reply = send_request(&session, &ControlRequest::DaemonConnected).unwrap();
    match reply {
        ControlRequestReply::DaemonConnected(connected) => {
            assert!(!connected, "no daemons should be connected");
        }
        other => panic!("expected DaemonConnected, got {other:?}"),
    }
}

#[test]
fn cli_stop_nonexistent() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    let fake_uuid = uuid::Uuid::new_v4();
    let reply = send_request(
        &session,
        &ControlRequest::Stop {
            dataflow_uuid: fake_uuid,
            grace_duration: None,
            force: false,
        },
    )
    .unwrap();

    match reply {
        ControlRequestReply::Error(msg) => {
            assert!(
                msg.contains(&fake_uuid.to_string())
                    || msg.to_lowercase().contains("not found")
                    || !msg.is_empty(),
                "error should be descriptive: {msg}"
            );
        }
        other => panic!("expected Error, got {other:?}"),
    }
}

#[test]
fn cli_multiple_requests_same_session() {
    let port = start_coordinator_background();
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr).expect("failed to connect WsSession");

    // First request: List
    let reply1 = send_request(&session, &ControlRequest::List).unwrap();
    assert!(matches!(reply1, ControlRequestReply::DataflowList(_)));

    // Second request: DaemonConnected
    let reply2 = send_request(&session, &ControlRequest::DaemonConnected).unwrap();
    assert!(matches!(
        reply2,
        ControlRequestReply::DaemonConnected(false)
    ));

    // Third request: ConnectedMachines
    let reply3 = send_request(&session, &ControlRequest::ConnectedMachines).unwrap();
    match reply3 {
        ControlRequestReply::ConnectedDaemons(daemons) => {
            assert!(daemons.is_empty());
        }
        other => panic!("expected ConnectedDaemons, got {other:?}"),
    }
}
