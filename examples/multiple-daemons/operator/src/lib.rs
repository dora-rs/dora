#![warn(unsafe_op_in_unsafe_fn)]

use adora_operator_api::{
    AdoraOperator, AdoraOutputSender, AdoraStatus, Event, IntoArrow, register_operator,
};

register_operator!(ExampleOperator);

#[derive(Debug, Default)]
struct ExampleOperator {
    ticks: usize,
}

impl AdoraOperator for ExampleOperator {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut AdoraOutputSender,
    ) -> Result<AdoraStatus, String> {
        match event {
            Event::Input { id, metadata: _, data } => match *id {
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
                    output_sender.send("status", output.into_arrow())?;
                }
                other => eprintln!("ignoring unexpected input {other}"),
            },
            Event::Stop => {}
            Event::InputClosed { id } => {
                println!("input `{id}` was closed");
                if *id == "random" {
                    println!("`random` input was closed -> exiting");
                    return Ok(AdoraStatus::Stop);
                }
            }
            other => {
                println!("received unknown event {other:?}");
            }
        }

        Ok(AdoraStatus::Continue)
    }
}
