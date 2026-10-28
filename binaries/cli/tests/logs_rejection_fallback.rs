//! `dora logs` when a coordinator answers on the port but refuses the CLI.
//!
//! The refusing coordinator may not be the one that ran the dataflow: a plain
//! `dora run` never uses one, but an older `dora up` left running across a CLI
//! upgrade, or another user's `--auth` coordinator, can still hold the port.
//! So `dora logs` must keep reading the local `out/` logs, and say why it
//! didn't use the coordinator. Only without local logs is the rejection the
//! error.

use std::{
    io::{Read, Write},
    net::{SocketAddr, TcpListener},
    path::Path,
    process::{Command, Output},
};

/// Answers every WS upgrade with `401 Unauthorized`, like a coordinator
/// started with `--auth` given a wrong token.
fn unauthorized_coordinator() -> SocketAddr {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = listener.local_addr().unwrap();
    std::thread::spawn(move || {
        for stream in listener.incoming() {
            let Ok(mut stream) = stream else { return };
            let mut buf = [0u8; 4096];
            let _ = stream.read(&mut buf);
            let _ = stream.write_all(b"HTTP/1.1 401 Unauthorized\r\nContent-Length: 0\r\n\r\n");
        }
    });
    addr
}

fn dora_logs(cwd: &Path, coordinator: SocketAddr, extra: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_dora"))
        .current_dir(cwd)
        .arg("logs")
        .args(extra)
        .arg("--coordinator-addr")
        .arg(coordinator.ip().to_string())
        .arg("--coordinator-port")
        .arg(coordinator.port().to_string())
        .env_remove("DORA_AUTH_TOKEN")
        .env_remove("DORA_LOG_LEVEL")
        .env_remove("DORA_LOG_FORMAT")
        .env_remove("DORA_LOG_FILTER")
        .output()
        .expect("failed to run `dora logs`")
}

/// An `out/` directory as a `dora run` of one `sensor` node leaves it.
fn dora_run_output(dir: &Path) {
    let dataflow = dir.join("out").join("0199f0a6-0000-7000-8000-000000000000");
    std::fs::create_dir_all(&dataflow).unwrap();
    std::fs::write(dataflow.join("log_sensor.txt"), "hello from dora run\n").unwrap();
}

#[test]
fn logs_fall_back_to_local_logs_when_the_coordinator_refuses_the_cli() {
    let coordinator = unauthorized_coordinator();
    let dir = tempfile::tempdir().unwrap();
    dora_run_output(dir.path());

    for extra in [&[][..], &["--node", "sensor"][..]] {
        let output = dora_logs(dir.path(), coordinator, extra);
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(output.status.success(), "{extra:?}\n{stdout}\n{stderr}");
        assert!(
            stdout.contains("hello from dora run"),
            "{extra:?}\n{stdout}\n{stderr}"
        );
        assert!(
            stderr.contains("reading local logs from out/ instead")
                && stderr.contains("authentication failed"),
            "{extra:?}\n{stderr}"
        );
    }
}

#[test]
fn logs_report_the_rejection_when_there_are_no_local_logs() {
    let coordinator = unauthorized_coordinator();
    let dir = tempfile::tempdir().unwrap();

    for extra in [&[][..], &["--node", "sensor"][..]] {
        let output = dora_logs(dir.path(), coordinator, extra);
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(!output.status.success(), "{extra:?}\n{stderr}");
        assert!(
            stderr.contains("authentication failed"),
            "{extra:?}\n{stderr}"
        );
        assert!(!stderr.contains("no out/ directory"), "{extra:?}\n{stderr}");
    }
}
