/*
    Basic TCP implementation for streaming events to a TCP Destination
*/
use dora_node_api::{DoraNode, Event};
use eyre::Result;
use tokio::io::AsyncWriteExt;
use tokio::net::TcpStream;

pub async fn lib_main() -> Result<()> {
    // NOTE: Hard-coded address
    let mut stream =
        TcpStream::connect(std::env::var("ADDR").unwrap_or("127.0.0.1:8052".to_string())).await?;

    let (_, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata: _,
                data,
            } => match id.as_str() {
                "text" => {
                    // Extract the command
                    let command: &str = (&data).try_into()?;

                    // Send the command to the Unity over tcp
                    stream.write_all(command.as_bytes()).await?;
                }
                _ => {}
            },
            _ => {}
        }
    }

    Ok(())
}
