//! Tests for `transport::connect` URL parsing and scheme handling.
//! Real network round-trip tests live in `binaries/mavlink2-bridge-node`.
//!
//! Note: `Box<dyn MavConnection<...>>` does not implement `Debug`, so
//! these tests use `match` rather than `Result::expect_err`.

use dora_mavlink2_bridge::{BridgeError, transport};
use url::Url;

fn assert_config_err_with(result: Result<impl Sized, BridgeError>, expected_substring: &str) {
    match result {
        Ok(_) => panic!("expected BridgeError::Config containing '{expected_substring}', got Ok"),
        Err(BridgeError::Config(m)) => assert!(
            m.contains(expected_substring),
            "expected error message to contain '{expected_substring}', got: {m}"
        ),
        Err(other) => panic!(
            "expected BridgeError::Config, got {} ({other})",
            std::any::type_name_of_val(&other)
        ),
    }
}

// ---- scheme rejection ----

#[test]
fn rejects_unknown_scheme() {
    let url = Url::parse("https://example.com").unwrap();
    assert_config_err_with(transport::connect(&url), "unsupported");
}

// ---- TCP ----

#[test]
fn tcp_with_no_host_rejects() {
    let url = Url::parse("tcp:5760").unwrap();
    assert!(url.host_str().is_none());
    assert_config_err_with(transport::connect(&url), "missing host");
}

#[test]
fn tcp_unreachable_endpoint_returns_config_error_not_panic() {
    let url = Url::parse("tcp://127.0.0.1:1").unwrap();
    assert_config_err_with(transport::connect(&url), "failed to connect");
}

#[test]
fn default_tcp_port_constant_is_5760() {
    assert_eq!(transport::DEFAULT_TCP_PORT, 5760);
}

// ---- UDP ----

#[test]
fn udp_with_no_host_rejects() {
    let url = Url::parse("udp:14550").unwrap();
    assert!(url.host_str().is_none());
    assert_config_err_with(transport::connect(&url), "missing host");
}

#[test]
fn default_udp_port_constant_is_14550() {
    assert_eq!(transport::DEFAULT_UDP_PORT, 14550);
}

// Note: a positive `udp://host:port` test that actually binds lives in
// `binaries/mavlink2-bridge-node/tests/udp_loopback.rs` — binding the
// port from a unit test would race with the loopback test if both run
// in parallel.

// ---- Serial ----

#[test]
fn serial_nonexistent_device_unix_form_returns_config_error() {
    // `/nonexistent/path/that/should/not/exist` cannot be opened. We
    // only assert the error wraps as Config, not the OS-specific text.
    let url = Url::parse("serial:///nonexistent/path/that/should/not/exist?baud=115200").unwrap();
    match transport::connect(&url) {
        Ok(_) => panic!("expected error opening nonexistent device"),
        Err(BridgeError::Config(_)) => {}
        Err(other) => panic!("expected BridgeError::Config, got {other}"),
    }
}

#[test]
fn serial_empty_device_path_rejects() {
    // `serial://` parses with empty host and empty path → caught as
    // "missing device path" by our parser.
    let url = Url::parse("serial://").unwrap();
    assert_config_err_with(transport::connect(&url), "missing device path");
}

#[test]
fn default_serial_baud_constant_is_115200() {
    assert_eq!(transport::DEFAULT_SERIAL_BAUD, 115_200);
}

// ---- tlog replay ----

#[cfg(not(feature = "tlog"))]
#[test]
fn tlog_without_feature_names_the_feature() {
    let url = Url::parse("tlog:///var/log/flight.tlog").unwrap();
    assert_config_err_with(transport::validate(&url), "`tlog` feature");
}

#[cfg(feature = "tlog")]
mod tlog {
    use super::assert_config_err_with;
    use dora_mavlink2_bridge::mavlink::{
        self, MavHeader, MavlinkVersion,
        dialects::common::{HEARTBEAT_DATA, MavMessage},
        error::MessageReadError,
    };
    use dora_mavlink2_bridge::transport;
    use std::path::{Path, PathBuf};
    use std::time::{Duration, Instant};
    use url::Url;

    /// Gap between the recorded HEARTBEATs in [`write_log`].
    const GAP_US: u64 = 150_000;

    /// Write a `.tlog` of three HEARTBEATs (sequence 0, 1, 2) recorded
    /// `GAP_US` apart, the second one a MAVLink 1 frame, with a few bytes of
    /// garbage between the first two records as a damaged log would have.
    fn write_log(name: &str) -> PathBuf {
        let path = std::env::temp_dir().join(format!(
            "dora-mavlink2-bridge-{}-{name}.tlog",
            std::process::id()
        ));
        let mut log = Vec::new();
        for (sequence, version) in [MavlinkVersion::V2, MavlinkVersion::V1, MavlinkVersion::V2]
            .into_iter()
            .enumerate()
        {
            let header = MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: sequence as u8,
            };
            let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
            log.extend_from_slice(
                &(1_700_000_000_000_000 + sequence as u64 * GAP_US).to_be_bytes(),
            );
            mavlink::write_versioned_msg(&mut log, version, header, &msg).unwrap();
            if sequence == 0 {
                log.extend_from_slice(&[0xFD, 0x00, 0xFE, 0x13, 0x37]);
            }
        }
        std::fs::write(&path, log).unwrap();
        path
    }

    fn tlog_url(path: &Path, query: &str) -> Url {
        let file_url = Url::from_file_path(path).unwrap();
        Url::parse(&format!("tlog://{}{query}", file_url.path())).unwrap()
    }

    fn recv_sequences(
        conn: &(dyn mavlink::MavConnection<MavMessage> + Send + Sync),
        count: usize,
    ) -> Vec<u8> {
        let mut sequences = Vec::new();
        while sequences.len() < count {
            match conn.recv() {
                Ok((header, MavMessage::HEARTBEAT(_))) => sequences.push(header.sequence),
                Ok((_, other)) => panic!("unexpected message {other:?}"),
                Err(MessageReadError::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => panic!("recv failed: {e}"),
            }
        }
        sequences
    }

    #[test]
    fn replays_every_record_at_the_recorded_pace() {
        let path = write_log("paced");
        let conn = match transport::connect(&tlog_url(&path, "")) {
            Ok(conn) => conn,
            Err(e) => panic!("opening the replay failed: {e}"),
        };
        let start = Instant::now();
        assert_eq!(recv_sequences(conn.as_ref(), 3), [0, 1, 2]);
        let elapsed = start.elapsed();
        assert!(
            elapsed >= Duration::from_micros(2 * GAP_US - 20_000),
            "three records 150 ms apart replayed in {elapsed:?}"
        );

        // The log is exhausted: the link goes quiet instead of failing.
        let start = Instant::now();
        match conn.recv() {
            Err(MessageReadError::Io(e)) => {
                assert_eq!(e.kind(), std::io::ErrorKind::WouldBlock);
            }
            Ok(_) => panic!("got a message after the end of the log"),
            Err(e) => panic!("expected WouldBlock after the end of the log, got {e}"),
        }
        assert!(start.elapsed() < Duration::from_secs(1));
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn speed_zero_replays_unpaced() {
        let path = write_log("unpaced");
        let conn = match transport::connect(&tlog_url(&path, "?speed=0")) {
            Ok(conn) => conn,
            Err(e) => panic!("opening the replay failed: {e}"),
        };
        let start = Instant::now();
        assert_eq!(recv_sequences(conn.as_ref(), 3), [0, 1, 2]);
        assert!(start.elapsed() < Duration::from_micros(GAP_US));
        std::fs::remove_file(path).unwrap();
    }

    /// A speed so small that the second record's due time overflows must
    /// leave that record pending, not panic.
    #[test]
    fn tiny_speed_does_not_overflow() {
        let path = write_log("tiny-speed");
        let conn = match transport::connect(&tlog_url(&path, "?speed=1e-300")) {
            Ok(conn) => conn,
            Err(e) => panic!("opening the replay failed: {e}"),
        };
        assert_eq!(recv_sequences(conn.as_ref(), 1), [0]);
        match conn.recv() {
            Err(MessageReadError::Io(e)) => assert_eq!(e.kind(), std::io::ErrorKind::WouldBlock),
            Ok(_) => panic!("the second record is not due for longer than `Instant` can hold"),
            Err(e) => panic!("expected WouldBlock, got {e}"),
        }
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn sending_to_a_replay_fails() {
        let path = write_log("send");
        let conn = match transport::connect(&tlog_url(&path, "")) {
            Ok(conn) => conn,
            Err(e) => panic!("opening the replay failed: {e}"),
        };
        let msg = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
        assert!(conn.send(&MavHeader::default(), &msg).is_err());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn missing_file_is_a_config_error() {
        let path = std::env::temp_dir().join("dora-mavlink2-bridge-no-such-log.tlog");
        assert_config_err_with(
            transport::connect(&tlog_url(&path, "")),
            "failed to open tlog replay",
        );
    }

    #[test]
    fn rejects_bad_speed_and_relative_path() {
        let path = std::env::temp_dir().join("flight.tlog");
        for speed in ["-1", "fast", "inf", "NaN", ""] {
            let url = tlog_url(&path, &format!("?speed={speed}"));
            assert_config_err_with(transport::validate(&url), "invalid tlog speed");
        }
        let url = Url::parse("tlog://flight.tlog").unwrap();
        assert_config_err_with(transport::validate(&url), "missing or relative log path");
        assert!(transport::validate(&tlog_url(&path, "?speed=2.5")).is_ok());
    }
}
