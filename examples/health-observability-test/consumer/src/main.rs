use dora_node_api::{DoraNode, Event};
use dora_core::config::NodeId;
use dora_message::common::HealthStatus;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    
    println!("[Consumer] Node started");
    
    // Subscribe to producer's lifecycle events
    node.subscribe_node_events(&[NodeId::from("producer".to_string())])?;
    println!("[Consumer] Subscribed to producer events");
    
    // Set initial health status
    node.set_health_status(HealthStatus::Healthy)?;
    println!("[Consumer] Set health status to Healthy");
    
    for event in events {
        match event {
            Event::Stop(_) => {
                println!("[Consumer] Received stop signal");
                break;
            }
            Event::Input { id, data, .. } => {
                println!("[Consumer] Received input on {}: {:?}", id, data);
                
                // Query input health
                let health = node.query_input_health(&id)?;
                println!("[Consumer] Input health status: {:?}", health);
            }
            Event::PeerStarted { node_id } => {
                println!("[Consumer] Peer started: {}", node_id);
            }
            Event::PeerStopped { node_id, reason } => {
                println!("[Consumer] Peer stopped: {} (reason: {})", node_id, reason);
            }
            Event::PeerHealthChanged { node_id, status } => {
                println!("[Consumer] Peer {} health changed to: {:?}", node_id, status);
            }
            Event::InputError { id, error } => {
                println!("[Consumer] Error on input {}: {}", id, error);
                // Handle error gracefully
                node.set_health_status(HealthStatus::Degraded)?;
                println!("[Consumer] Set own health to Degraded due to upstream error");
            }
            _ => {}
        }
    }
    
    println!("[Consumer] Exiting");
    Ok(())
}

