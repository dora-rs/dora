#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{register_operator, DoraOperator, DoraOutputSender, DoraStatus};
use std::time::{Duration, Instant};

register_operator!(ExampleOperator);

#[derive(Debug, Default)]
struct ExampleOperator {
    ticks: usize,
    last_random_at: Option<Instant>,
}

impl DoraOperator for ExampleOperator {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, String> {
        match id {
            "tick" => {
                self.ticks += 1;
            }
            "random" => {
                let parsed = {
                    let data: [u8; 8] = data.try_into().map_err(|_| "unexpected random data")?;
                    u64::from_le_bytes(data)
                };
                let data = format!(
                    "operator received random value {parsed} after {} ticks",
                    self.ticks
                );
                let mut output = output_sender.prepare("status".into(), data.len())?;
                output.data_mut().copy_from_slice(data.as_bytes());
                output.send()?;
                self.last_random_at = Some(Instant::now());
            }
            other => eprintln!("ignoring unexpected input {other}"),
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
