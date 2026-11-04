use dora_node_api::{
    DoraNode,
    integration_testing::{
        IntegrationTestInput, TestingInput, TestingOutput,
        integration_testing_format::{IncomingEvent, TimedIncomingEvent},
        output_channel,
    },
};
use std::{sync::mpsc, time::Duration};

const INITIALIZATION_ATTEMPTS: usize = 64;
const INITIALIZATION_DEADLINE: Duration = Duration::from_secs(2);
const SCHEDULED_EVENT_SECONDS: f64 = 30.0;

#[test]
fn initialization_does_not_wait_for_scheduled_input() {
    for _ in 0..INITIALIZATION_ATTEMPTS {
        // Arrange
        let (initialized, ready) = mpsc::channel();
        let worker = std::thread::spawn(move || {
            let input = IntegrationTestInput::new(
                "initialization-test".parse().unwrap(),
                vec![TimedIncomingEvent {
                    time_offset_secs: SCHEDULED_EVENT_SECONDS,
                    event: IncomingEvent::Stop,
                }],
            );
            let (output, _outputs) = output_channel();

            // Act
            let (node, events) = DoraNode::init_testing(
                TestingInput::Input(input),
                TestingOutput::ToChannel(output),
                Default::default(),
            )
            .unwrap();
            initialized.send(()).unwrap();
            drop(events);
            drop(node);
        });

        // Assert: constructing the node must finish before its future input arrives.
        ready
            .recv_timeout(INITIALIZATION_DEADLINE)
            .expect("node initialization waited for a scheduled event");
        worker.join().unwrap();
    }
}
