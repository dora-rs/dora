#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{register_operator, DoraOperator, DoraOutputSender, DoraStatus};

register_operator!(ExampleOperator);

#[derive(Debug, Default)]
struct ExampleOperator {
    time: Option<String>,
}

impl DoraOperator for ExampleOperator {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, ()> {
        match id {
            "time" => {
                let parsed = std::str::from_utf8(data).map_err(|_| ())?;
                self.time = Some(parsed.to_owned());
            }
            "random" => {
                let parsed = {
                    let data: [u8; 8] = data.try_into().map_err(|_| ())?;
                    u64::from_le_bytes(data)
                };
                if let Some(time) = &self.time {
                    let output = format!("state operator random value {parsed} at {time}");
                    output_sender
                        .send("timestamped-random", output.as_bytes())
                        .map_err(|_| ())?;
                }
            }
            other => eprintln!("ignoring unexpected input {other}"),
        }
        Ok(DoraStatus::Continue)
    }
}
