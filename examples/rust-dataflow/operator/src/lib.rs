#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{
    register_operator,
    types::arrow::{
        array::{AsArray, PrimitiveArray},
        datatypes::UInt64Type,
    },
    DoraOperator, DoraOutputSender, DoraStatus, Event, IntoArrow,
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
                    let primitive_array: &PrimitiveArray<UInt64Type> =
                        data.as_primitive_opt().ok_or("expected primitive array")?;
                    let value = primitive_array.value(0);

                    let output = format!(
                        "operator received random value {value:#x} after {} ticks",
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
