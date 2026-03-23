use adora_operator_api::{register_operator, AdoraOperator, AdoraOutputSender, AdoraStatus, Event};

register_operator!(ExampleOperator);

#[derive(Debug, Default)]
struct ExampleOperator {
    example_field: u32,
}

impl AdoraOperator for ExampleOperator {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut AdoraOutputSender,
    ) -> Result<AdoraStatus, String> {
        match event {
            Event::Input { id, metadata: _, data: _ } => match id {
                other => eprintln!("Received input {other}"),
            },
            _other => {}
        }

        Ok(AdoraStatus::Continue)
    }
}
