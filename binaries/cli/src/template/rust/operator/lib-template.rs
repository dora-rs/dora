use dora_operator_api::{register_operator, DoraOperator, DoraOutputSender, DoraStatus, Event};

register_operator!(ExampleOperator);

#[derive(Debug, Default)]
struct ExampleOperator {
    example_field: u32,
}

impl DoraOperator for ExampleOperator {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, String> {
        match event {
            Event::Input { id, metadata: _, data: _ } => match id {
                other => eprintln!("Received input {other}"),
            },
            _other => {}
        }

        Ok(DoraStatus::Continue)
    }
}
