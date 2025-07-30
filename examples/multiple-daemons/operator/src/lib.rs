#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{
    DoraOperator, DoraOutputSender, DoraStatus, Event, IntoArrow, register_operator,
};

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
                    let data = u64::try_from(data)
                        .map_err(|err| format!("expected u64 message: {err}"))?;

                    let output = format!(
                        "operator received random value {data:#x} after {} ticks",
                        self.ticks
                    );
                    output_sender.send("status".into(), output.into_arrow())?;
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
