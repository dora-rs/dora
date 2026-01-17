use dora_node_api::{
    self, DoraNode, Event, IntoArrow, dora_core::config::DataId, node_failure_error,
};
use eyre::Context;

fn main() -> eyre::Result<()> {
    println!("hello");

    let status_output = DataId::from("status".to_owned());
    let (mut node, mut events) = DoraNode::init_from_env()?;

    let mut ticks = 0;
    let mut last_random = 0u64;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_ref() {
                "tick" => {
                    ticks += 1;
                }
                "random" => {
                    let value = u64::try_from(&data).context("unexpected data type")?;
                    last_random = value;

                    let output =
                        format!("operator received random value {value:#x} after {ticks} ticks");
                    node.send_output(
                        status_output.clone(),
                        metadata.parameters,
                        output.into_arrow(),
                    )?;
                }
                "fail" => {
                    if last_random % 2 == 0 {
                        // use panic to simulate a failure
                        panic!("simulated failure through panic");
                    } else {
                        // use report function to simulate a failure
                        let mut error =
                            node_failure_error!("simulated failure through report_failure_error");
                        error.detailed =
                            Some("simulated failure after receiving 'fail' input".into());
                        node.report_failure_error(error)?;
                        return Err(eyre::eyre!("simulated failure"));
                    }
                }
                "trigger-exit" => {
                    println!("trigger-exit received, sending exit signal to sink");
                    node.send_output_bytes("triggered-exit".into(), metadata.parameters, 0, &[])?
                }
                other => eprintln!("ignoring unexpected input {other}"),
            },
            Event::Stop(_) => {}
            Event::InputClosed { id } => {
                println!("input `{id}` was closed");
                if *id == "random" {
                    println!("`random` input was closed -> exiting");
                    break;
                }
            }
            other => {
                println!("received unknown event {other:?}");
            }
        }
    }

    Ok(())
}
