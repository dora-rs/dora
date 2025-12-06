use dora_node_api::{DoraNode, Event};
use eyre::Result;

fn main() -> Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    println!("[Consumer] Starting...");

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, .. } => {
                println!("[Consumer] Received input on {}", id);
            }
            Event::InputError {
                id,
                error,
                source_node_id,
            } => {
                println!(
                    "[Consumer] ⚠️  Received error from node '{}' on input '{}': {}",
                    source_node_id, id, error
                );
                println!("[Consumer] Handling error gracefully - using cached data...");
                // In a real application, you could:
                // - Use cached/backup data
                // - Switch to alternative input source
                // - Log the error and continue
                // - Implement retry logic
            }
            Event::InputClosed { id } => {
                println!("[Consumer] Input {} closed", id);
            }
            Event::Stop(_) => {
                println!("[Consumer] Received stop signal");
                break;
            }
            _ => {}
        }
    }

    println!("[Consumer] Exiting");
    Ok(())
}

