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
