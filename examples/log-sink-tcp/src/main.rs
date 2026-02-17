use adora_node_api::{AdoraNode, Event};
use eyre::Result;
use std::{
    env,
    io::Write,
    net::TcpStream,
};

fn main() -> Result<()> {
    let (_node, mut events) = AdoraNode::init_from_env()?;

    let addr = env::var("SINK_ADDR").unwrap_or_else(|_| "127.0.0.1:9876".to_string());
    let mut stream = connect(&addr);

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id: _, metadata: _, data } => {
                let log = match adora_log_utils::parse_log_from_arrow(&data) {
                    Ok(log) => log,
                    Err(e) => {
                        eprintln!("failed to parse log entry: {e}");
                        continue;
                    }
                };

                let line = format!("{}\n", adora_log_utils::format_json(&log));

                // Write to TCP stream, reconnect on failure
                if let Some(ref mut s) = stream {
                    if s.write_all(line.as_bytes()).is_err() {
                        eprintln!("TCP write failed, reconnecting...");
                        stream = connect(&addr);
                        if let Some(ref mut s) = stream {
                            let _ = s.write_all(line.as_bytes());
                        }
                    }
                } else {
                    stream = connect(&addr);
                    if let Some(ref mut s) = stream {
                        let _ = s.write_all(line.as_bytes());
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

fn connect(addr: &str) -> Option<TcpStream> {
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
