#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{register_operator, DoraContext, DoraOperator, DoraStatus};
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
        dora_context: &mut DoraContext,
    ) -> Result<DoraStatus, ()> {
        match id {
            "tick" => {
                self.ticks += 1;
                dbg!(dora_context.opentelemetry_context());
            }
            "random" => {
                let parsed = {
                    let data: [u8; 8] = data.try_into().map_err(|_| ())?;
                    u64::from_le_bytes(data)
                };
                let output = format!(
                    "operator received random value {parsed} after {} ticks",
                    self.ticks
                );
                dora_context
                    .send_output("status", output.as_bytes())
                    .map_err(|_| ())?;
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
