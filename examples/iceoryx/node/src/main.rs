use dora_node_api::{self, config::DataId, DoraNode};

fn main() -> eyre::Result<()> {
    let output = DataId::from("random".to_owned());

    let mut operator = DoraNode::init_from_env()?;

    let inputs = operator.inputs()?;

    for _ in 0..20 {
        let input = match inputs.recv() {
            Ok(input) => input,
            Err(_) => break,
        };

        match input.id.as_str() {
            "tick" => {
                let random: u64 = rand::random();
                operator.send_output(&output, &random.to_le_bytes())?;
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    Ok(())
}
