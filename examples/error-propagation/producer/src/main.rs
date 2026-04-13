use dora_node_api::{DoraNode, IntoArrow, MetadataParameters};
use eyre::Result;

fn main() -> Result<()> {
    let (mut node, _events) = DoraNode::init_from_env()?;

    println!("Starting...");

    for i in 0u8..3 {
        node.send_output("data".into(), MetadataParameters::default(), i.into_arrow())?;
        println!("Sent message {i}");
        std::thread::sleep(std::time::Duration::from_secs(1));
    }

    println!("Encountered an error! Exiting with error code...");
    eprintln!("Error: Simulated processing failure");
    std::process::exit(1);
}
