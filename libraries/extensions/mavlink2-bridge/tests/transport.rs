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

#[test]
fn rejects_unknown_scheme() {
    let url = Url::parse("https://example.com").unwrap();
    assert_config_err_with(transport::connect(&url), "unsupported");
}

#[test]
fn rejects_udp_until_pr4() {
    // PR #4 adds udp/serial; this PR must explicitly reject them so the
    // user gets a clear error rather than a panic.
    let url = Url::parse("udp://0.0.0.0:14550").unwrap();
    assert_config_err_with(transport::connect(&url), "unsupported");
}

#[test]
fn tcp_with_no_host_rejects() {
    // A `tcp:` URL with no authority component has `host_str() == None`.
    let url = Url::parse("tcp:5760").unwrap();
    assert!(url.host_str().is_none());
    assert_config_err_with(transport::connect(&url), "missing host");
}

#[test]
fn tcp_unreachable_endpoint_returns_config_error_not_panic() {
    // 127.0.0.1:1 has no listener — the connect attempt fails. We only
    // care that the error is wrapped, not that connect succeeded.
    let url = Url::parse("tcp://127.0.0.1:1").unwrap();
    assert_config_err_with(transport::connect(&url), "failed to connect");
}

#[test]
fn default_tcp_port_constant_is_5760() {
    assert_eq!(transport::DEFAULT_TCP_PORT, 5760);
}
