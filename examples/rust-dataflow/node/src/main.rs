use dora_node_api::{self, core::config::DataId, DoraNode};

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
                let data: &[u8] = &random.to_le_bytes();
                operator.send_output(
                    &output,
                    input.metadata().parameters.clone(),
                    data.len(),
                    |out| {
                        out.copy_from_slice(data);
                    },
                )?;
            }
            other => eprintln!("Ignoring unexpected input `{other}`"),
        }
    }

    Ok(())
}
