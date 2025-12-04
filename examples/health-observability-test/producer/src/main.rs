use dora_node_api::{DoraNode, Event};
use dora_message::common::HealthStatus;
use std::time::Duration;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    
    println!("[Producer] Node started");
    
    // Set initial health status
    node.set_health_status(HealthStatus::Healthy)?;
    println!("[Producer] Set health status to Healthy");
    
    let mut count = 0;
    
    for event in events {
        match event {
            Event::Stop(_) => {
                println!("[Producer] Received stop signal");
                break;
            }
            Event::Input { .. } => {
                // Producer doesn't have inputs, but handle it anyway
            }
            _ => {}
        }
        
        // Send some data
        if count < 5 {
            let data = format!("Message {}", count);
            println!("[Producer] Sending: {}", data);
            
            node.send_output(
                "data".into(),
                Default::default(),
                arrow::array::StringArray::from(vec![data]).into(),
            )?;
            
            count += 1;
            std::thread::sleep(Duration::from_millis(500));
        } else {
            // After 5 messages, declare degraded status and send error
            node.set_health_status(HealthStatus::Degraded)?;
            println!("[Producer] Set health status to Degraded");
            
            node.send_error("data".into(), "Simulated error condition")?;
            println!("[Producer] Sent error event");
            
            std::thread::sleep(Duration::from_secs(1));
            break;
        }
    }
    
    println!("[Producer] Exiting");
    Ok(())
}

