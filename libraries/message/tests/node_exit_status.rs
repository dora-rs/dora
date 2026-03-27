//! Invariant tests for NodeExitStatus classification.
//!
//! These tests encode the expected behavior of the `From<Result<ExitStatus, io::Error>>`
//! implementation for `NodeExitStatus`. They ensure that process termination states are
//! correctly classified across platforms (Unix and Windows).
//!
//! No production code is modified; these are purely defensive tests to prevent regressions.

use dora_message::common::NodeExitStatus;
use std::io;
use std::process::ExitStatus;

/// Test that exit code 0 always maps to NodeExitStatus::Success
#[test]
fn exit_code_zero_is_success() {
    let status = create_exit_status(0);
    let node_status: NodeExitStatus = Ok(status).into();

    assert!(
        matches!(node_status, NodeExitStatus::Success),
        "Exit code 0 must map to Success, got: {:?}",
        node_status
    );
}

/// Test that non-zero exit codes map to NodeExitStatus::ExitCode(code)
#[test]
fn non_zero_exit_codes() {
    let test_cases = vec![1, 2, 127, 255];

    for code in test_cases {
        let status = create_exit_status(code);
        let node_status: NodeExitStatus = Ok(status).into();

        assert!(
            matches!(node_status, NodeExitStatus::ExitCode(c) if c == code),
            "Exit code {} must map to ExitCode({}), got: {:?}",
            code,
            code,
            node_status
        );
    }
}

/// Test that I/O errors map to NodeExitStatus::IoError
#[test]
fn io_error_maps_to_io_error_variant() {
    let io_err = io::Error::new(io::ErrorKind::BrokenPipe, "test error");
    let node_status: NodeExitStatus = Err(io_err).into();

    assert!(
        matches!(node_status, NodeExitStatus::IoError(_)),
        "io::Error must map to IoError variant, got: {:?}",
        node_status
    );
}

/// Unix-only: Test that signal termination maps to NodeExitStatus::Signal(signal)
#[cfg(unix)]
#[test]
fn unix_signal_termination() {
    use std::os::unix::process::ExitStatusExt;

    let test_signals = vec![(9, "SIGKILL"), (15, "SIGTERM"), (11, "SIGSEGV")];

    for (signal, name) in test_signals {
        // On Unix, when a process is terminated by a signal, the signal number
        // is stored in the lower 7 bits (0-127), and the process did NOT exit normally.
        // Use from_raw with just the signal number (no shift).
        let status = ExitStatus::from_raw(signal);
        let node_status: NodeExitStatus = Ok(status).into();

        assert!(
            matches!(node_status, NodeExitStatus::Signal(s) if s == signal),
            "Signal {} ({}) must map to Signal({}), got: {:?}",
            signal,
            name,
            signal,
            node_status
        );
    }
}

/// Test that NodeExitStatus::Unknown is not produced for valid inputs
///
/// This test verifies that for all standard process termination scenarios,
/// we never produce Unknown. Unknown should only occur in truly exceptional
/// circumstances that we cannot currently reproduce in tests.
#[test]
fn unknown_is_not_produced_for_valid_inputs() {
    // Test various exit codes
    for code in 0..=10 {
        let status = create_exit_status(code);
        let node_status: NodeExitStatus = Ok(status).into();
        assert!(
            !matches!(node_status, NodeExitStatus::Unknown),
            "Exit code {} should not produce Unknown, got: {:?}",
            code,
            node_status
        );
    }

    // Test I/O error
    let io_err = io::Error::new(io::ErrorKind::Other, "test");
    let node_status: NodeExitStatus = Err(io_err).into();
    assert!(
        !matches!(node_status, NodeExitStatus::Unknown),
        "I/O error should not produce Unknown"
    );

    // Unix: Test signals
    #[cfg(unix)]
    {
        use std::os::unix::process::ExitStatusExt;
        for signal in [1, 2, 9, 15] {
            // Signals are stored in lower 7 bits for signal termination
            let status = ExitStatus::from_raw(signal);
            let node_status: NodeExitStatus = Ok(status).into();
            assert!(
                !matches!(node_status, NodeExitStatus::Unknown),
                "Signal {} should not produce Unknown, got: {:?}",
                signal,
                node_status
            );
        }
    }
}

// Platform-specific helper to create ExitStatus values for testing

#[cfg(unix)]
fn create_exit_status(code: i32) -> ExitStatus {
    use std::os::unix::process::ExitStatusExt;
    // On Unix, exit code is stored in bits 8-15
    ExitStatus::from_raw(code << 8)
}

#[cfg(windows)]
fn create_exit_status(code: i32) -> ExitStatus {
    use std::os::windows::process::ExitStatusExt;
    // On Windows, use from_raw directly with the exit code
    ExitStatus::from_raw(code as u32)
}

#[cfg(not(any(unix, windows)))]
fn create_exit_status(_code: i32) -> ExitStatus {
    compile_error!("Unsupported platform for exit status tests");
}
