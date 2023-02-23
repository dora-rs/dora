#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{register_operator, DoraOperator, DoraOutputSender, DoraStatus, Event};
use std::time::{Duration, Instant};

register_operator!(ExampleOperator);

#[derive(Debug, Default)]
struct ExampleOperator {
    ticks: usize,
    last_random_at: Option<Instant>,
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
                    self.last_random_at = Some(Instant::now());
                }
                other => eprintln!("ignoring unexpected input {other}"),
            },
            Event::Stop => {}
        }

        if let Some(last_random_at) = self.last_random_at {
            if last_random_at.elapsed() > Duration::from_secs(1) {
                // looks like the node sending the random values finished -> exit too
                return Ok(DoraStatus::Stop);
            }
        }
        Ok(DoraStatus::Continue)
    }
}
