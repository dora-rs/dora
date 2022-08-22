#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{
    register_operator, DataId, DoraContext, DoraOperator, DoraStatus, Metadata,
};
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
        metadata: &Metadata,
        data: &[u8],
        dora_context: &mut DoraContext,
    ) -> Result<DoraStatus, ()> {
        match metadata.id.as_str() {
            "tick" => {
                self.ticks += 1;
                dbg!(&metadata.otel_context);
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
                    .send_output(
                        Metadata {
                            id: DataId::from("status".to_string()),
                            otel_context: metadata.otel_context.clone(),
                        },
                        output.as_bytes(),
                    )
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
