use dora_node_api::{
    self,
    dora_core::{config::DataId, daemon_messages::NodeEvent},
    DoraNode,
};

fn main() -> eyre::Result<()> {
    let output = DataId::from("random".to_owned());

    let (mut node, events) = DoraNode::init_from_env()?;

    for _ in 0..20 {
        let event = match events.recv() {
            Ok(input) => input,
            Err(_) => break,
        };

        match event {
            NodeEvent::Stop => break,
            NodeEvent::Input {
                id,
                metadata,
                data: _,
            } => match id.as_str() {
                "tick" => {
                    let random: u64 = rand::random();
                    let data: &[u8] = &random.to_le_bytes();
                    node.send_output(output.clone(), metadata.parameters, data.len(), |out| {
                        out.copy_from_slice(data);
                    })?;
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
        }
    }

    Ok(())
}
