/*
    Basic TCP implementation for streaming events to a TCP Destination
*/
use dora_node_api::{DoraNode, Event};
use tokio::io::AsyncWriteExt;
use tokio::net::TcpStream;
use std::error::Error;


#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {

    // NOTE: Hard-coded address
    let mut stream = TcpStream::connect("localhost:8052").await?;

    let (_, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata: _,
                data,
            } => match id.as_str() {
                "command" => {
                    
                    // Extract the command
                    let command: &str = (&data).try_into()?;

                    // Send the command to the Unity over tcp
                    stream.write_all(command.as_bytes()).await?;
                },
                _ => {}
            },
            _ => {}
        }
    }

    Ok(())
}
