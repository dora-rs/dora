//! UDP loopback round-trip: a listener thread opens `udp://...`
//! through `transport::connect` (which maps to mavlink's `udpin:`),
//! and the test main thread sends one HEARTBEAT via raw mavlink
//! `udpout:`. Verifies the public transport API plumbs UDP correctly.
//!
//! The serial transport is symmetrical but cannot be exercised in
//! the test process without `socat` or hardware — manual smoke is
//! covered in the README and in PR #6.

use std::net::UdpSocket;
use std::time::Duration;

use dora_mavlink2_bridge::{
    mavlink::common::{HEARTBEAT_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType},
    transport,
};
use mavlink::{MavHeader, MavlinkVersion};
use url::Url;

/// Reserve a free UDP port and immediately drop the socket so the
/// mavlink listener can rebind it. The race window is microseconds.
fn pick_free_udp_port() -> u16 {
    let probe = UdpSocket::bind("127.0.0.1:0").expect("bind probe");
    let port = probe.local_addr().expect("local_addr").port();
    drop(probe);
    port
}

#[test]
fn transport_connect_receives_heartbeat_over_udp() -> Result<(), Box<dyn std::error::Error>> {
    let port = pick_free_udp_port();

    // Listener: bridge-side, uses `transport::connect` so this exercises
    // the public path the daemon-spawned bridge node would take.
    let listener_url = Url::parse(&format!("udp://127.0.0.1:{port}"))?;
    let listener = std::thread::Builder::new()
        .name("mavlink-udp-listener".into())
        .spawn(
            move || -> Result<HEARTBEAT_DATA, Box<dyn std::error::Error + Send + Sync>> {
                let conn = transport::connect(&listener_url).map_err(
                    |e| -> Box<dyn std::error::Error + Send + Sync> { e.to_string().into() },
                )?;
                let (_header, msg) =
                    conn.recv()
                        .map_err(|e| -> Box<dyn std::error::Error + Send + Sync> {
                            e.to_string().into()
                        })?;
                match msg {
                    MavMessage::HEARTBEAT(d) => Ok(d),
                    other => Err(format!("expected HEARTBEAT, got {other:?}").into()),
                }
            },
        )?;

    // Give the listener time to bind. UDP `udpin` blocks in `recv_from`
    // immediately after bind, so 200ms is plenty.
    std::thread::sleep(Duration::from_millis(200));

    // Sender: test-side, raw mavlink udpout (the side a Pixhawk would
    // typically be in if it were sending telemetry to a known host).
    let mut sender = mavlink::connect::<MavMessage>(&format!("udpout:127.0.0.1:{port}"))?;
    sender.set_protocol_version(MavlinkVersion::V2);

    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };
    let heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 99,
        mavtype: MavType::MAV_TYPE_QUADROTOR,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });
    sender.send(&header, &heartbeat)?;

    let received = listener
        .join()
        .expect("listener thread panicked")
        .map_err(|e| -> Box<dyn std::error::Error> { e.to_string().into() })?;

    assert_eq!(received.custom_mode, 99);
    assert_eq!(received.mavtype, MavType::MAV_TYPE_QUADROTOR);
    assert_eq!(received.system_status, MavState::MAV_STATE_ACTIVE);
    assert_eq!(received.mavlink_version, 3);
    Ok(())
}
