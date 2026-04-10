use dora_node_api::{DoraNode, Event};
use eyre::Result;
use std::{
    env,
    io::Write,
    net::{SocketAddr, TcpStream},
    time::{Duration, Instant},
};

/// Minimum delay between reconnect attempts to avoid SYN-flooding.
const RECONNECT_DELAY: Duration = Duration::from_secs(2);

fn main() -> Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    // Default matches dataflow.yml; override via SINK_ADDR env var.
    let raw_addr = env::var("SINK_ADDR").unwrap_or_else(|_| "127.0.0.1:9876".to_string());
    let addr: SocketAddr = raw_addr
        .parse()
        .map_err(|_| eyre::eyre!("SINK_ADDR is not a valid socket address: {raw_addr}"))?;

    let mut stream = connect(addr);
    let mut last_reconnect = Instant::now();

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id: _,
                metadata: _,
                data,
            } => {
                let log = match dora_log_utils::parse_log_from_arrow(&data) {
                    Ok(log) => log,
                    Err(e) => {
                        eprintln!("failed to parse log entry: {e}");
                        continue;
                    }
                };

                let line = format!("{}\n", dora_log_utils::format_json(&log));

                if !try_write(&mut stream, line.as_bytes()) {
                    // Throttle reconnects to avoid SYN-flooding
                    if last_reconnect.elapsed() >= RECONNECT_DELAY {
                        eprintln!("TCP write failed, reconnecting...");
                        stream = connect(addr);
                        last_reconnect = Instant::now();
                        if !try_write(&mut stream, line.as_bytes()) {
                            eprintln!("entry dropped after failed reconnect");
                        }
                    } else {
                        eprintln!("entry dropped (reconnect throttled)");
                    }
                }
            }
            Event::Stop(_) => break,
            Event::InputClosed { id } => {
                eprintln!("input `{id}` closed");
            }
            _ => {}
        }
    }

    Ok(())
}

fn try_write(stream: &mut Option<TcpStream>, data: &[u8]) -> bool {
    if let Some(s) = stream.as_mut() {
        if s.write_all(data).is_ok() {
            return true;
        }
        *stream = None;
    }
    false
}

fn connect(addr: SocketAddr) -> Option<TcpStream> {
    match TcpStream::connect(addr) {
        Ok(s) => {
            eprintln!("connected to {addr}");
            Some(s)
        }
        Err(e) => {
            eprintln!("failed to connect to {addr}: {e}");
            None
        }
    }
}
