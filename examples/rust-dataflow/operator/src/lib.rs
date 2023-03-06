#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{register_operator, DoraOperator, DoraOutputSender, DoraStatus, Event};

register_operator!(ExampleOperator);

#[derive(Debug, Default)]
struct ExampleOperator {
    ticks: usize,
}

impl DoraOperator for ExampleOperator {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, String> {
        match event {
            Event::Input { id, data } => match *id {
                "tick" => {
                    self.ticks += 1;
                }
                "random" => {
                    let parsed = {
                        let data: [u8; 8] =
                            (*data).try_into().map_err(|_| "unexpected random data")?;
                        u64::from_le_bytes(data)
                    };
                    let output = format!(
                        "operator received random value {parsed:#x} after {} ticks",
                        self.ticks
                    );
                    output_sender.send("status".into(), output.into_bytes())?;
                }
                other => eprintln!("ignoring unexpected input {other}"),
            },
            Event::Stop => {}
            Event::InputClosed { id } => {
                println!("input `{id}` was closed");
                if *id == "random" {
                    println!("`random` input was closed -> exiting");
                    return Ok(DoraStatus::Stop);
                }
            }
            other => {
                println!("received unknown event {other:?}");
            }
        }

        Ok(DoraStatus::Continue)
    }
}
