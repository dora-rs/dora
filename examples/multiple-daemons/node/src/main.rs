use adora_node_api::{self, AdoraNode, Event, IntoArrow, adora_core::config::DataId};

fn main() -> eyre::Result<()> {
    println!("hello");

    let output = DataId::from("random".to_owned());

    let (mut node, mut events) = AdoraNode::init_from_env()?;

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
            Event::Stop(_) => println!("Received stop"),
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    Ok(())
}
