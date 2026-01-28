use dora_node_api::{DoraNode, Event};
use eyre::Result;

fn main() -> Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    println!("Starting...");

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, .. } => {
                println!("Received input on {}", id);
            }
            Event::NodeFailed {
                affected_input_ids,
                error,
                source_node_id,
            } => {
                println!(
                    "⚠️  Received error from node '{}' affecting inputs {:?}: {}",
                    source_node_id, affected_input_ids, error
                );
                println!("Handling error in some way...");
                // In a real application, you could:
                // - Use cached/backup data
                // - Switch to alternative input source
                // - Log the error and continue
                // - Implement retry logic
            }
            Event::InputClosed { id } => {
                println!("Input {} closed", id);
            }
            Event::Stop(_) => {
                println!("Received stop signal");
                break;
            }
            _ => {}
        }
    }

    println!("Exiting");
    Ok(())
}
