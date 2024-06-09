use dora_node_api::{self, dora_core::config::DataId, DoraNode, Event, IntoArrow};

fn main() -> eyre::Result<()> {
    println!("hello");

    let output = DataId::from("random".to_owned());

    let Ok((mut node, mut events)) = DoraNode::init_from_env() else {
        println!("Could not initiate dora node");
        return Ok(());
    };

    for i in 0..100 {
        let event = match events.recv() {
            Some(input) => input,
            None => break,
        };

        match event {
            Event::Input {
                id,
                metadata,
                data: _,
            } => match id.as_str() {
                "tick" => {
                    let random: u64 = rand::random();
                    println!("tick {i}, sending {random:#x}");
                    node.send_output(output.clone(), metadata.parameters, random.into_arrow())?;
                }
                other => eprintln!("Ignoring unexpected input `{other}`"),
            },
            Event::Stop => println!("Received manual stop"),
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    Ok(())
}
