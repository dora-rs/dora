//! TCP loopback round-trip: a server thread accepts a MAVLink 2 client
//! connection (via mavlink's `tcpin:`) and sends a HEARTBEAT; the client
//! is created through `dora_mavlink2_bridge::transport::connect` so this
//! test exercises the public transport API rather than mavlink directly.
//!
//! No dora runtime is required — this asserts that the transport plumbs
//! real bytes correctly. The full bridge-node-in-a-dataflow smoke test
//! lives in PR #6 (#1786).

use std::net::TcpListener;
use std::time::Duration;

use dora_mavlink2_bridge::{
    mavlink::common::{HEARTBEAT_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType},
    transport,
};
use mavlink::{MavHeader, MavlinkVersion};
use url::Url;

/// Reserve a free port on 127.0.0.1 and immediately drop the listener so
/// the mavlink server can rebind it. The race window is microseconds.
fn pick_free_port() -> u16 {
    let probe = TcpListener::bind("127.0.0.1:0").expect("bind probe");
    let port = probe.local_addr().expect("local_addr").port();
    drop(probe);
    port
}

#[test]
fn transport_connect_receives_heartbeat_over_tcp() -> Result<(), Box<dyn std::error::Error>> {
    let port = pick_free_port();

    let server = std::thread::Builder::new()
        .name("mavlink-tcp-server".into())
        .spawn(
            move || -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
                // `tcpin:` binds + blocks in accept(); returns once the
                // client connects, then we send one HEARTBEAT and exit.
                let mut conn = mavlink::connect::<MavMessage>(&format!("tcpin:127.0.0.1:{port}"))?;
                conn.set_protocol_version(MavlinkVersion::V2);

                let header = MavHeader {
                    system_id: 1,
                    component_id: 1,
                    sequence: 0,
                };
                let heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                    custom_mode: 7,
                    mavtype: MavType::MAV_TYPE_QUADROTOR,
                    autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
                    base_mode: MavModeFlag::empty(),
                    system_status: MavState::MAV_STATE_ACTIVE,
                    mavlink_version: 3,
                });
                conn.send(&header, &heartbeat)?;
                Ok(())
            },
        )?;

    // Give the server a moment to enter accept().
    std::thread::sleep(Duration::from_millis(150));

    let url = Url::parse(&format!("tcp://127.0.0.1:{port}"))?;
    let client = transport::connect(&url)?;
    let (_header, msg) = client.recv()?;

    match msg {
        MavMessage::HEARTBEAT(d) => {
            assert_eq!(d.custom_mode, 7);
            assert_eq!(d.mavtype, MavType::MAV_TYPE_QUADROTOR);
            assert_eq!(d.system_status, MavState::MAV_STATE_ACTIVE);
            assert_eq!(d.mavlink_version, 3);
        }
        other => panic!("expected HEARTBEAT, got {other:?}"),
    }

    server
        .join()
        .expect("server thread panicked")
        .map_err(|e| -> Box<dyn std::error::Error> { e.to_string().into() })?;
    Ok(())
}
