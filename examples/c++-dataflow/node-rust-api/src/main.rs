use dora_node_api::{self, DoraNode, Input};
use futures::{executor::block_on, Stream, StreamExt};
use std::pin::Pin;

#[cxx::bridge]
mod ffi {
    struct DoraInput {
        end_of_input: bool,
        id: String,
        data: Vec<u8>,
    }

    struct DoraResult {
        error: String,
    }

    extern "Rust" {
        type Inputs<'a>;
        type OutputSender<'a>;

        fn next_input(inputs: &mut Inputs) -> DoraInput;
        fn send_output(output_sender: &OutputSender, id: String, data: &[u8]) -> DoraResult;
    }

    unsafe extern "C++" {
        include!("cxx-dataflow-example-node-rust-api/src/main.h");

        fn cxx_main(inputs: &mut Inputs, output_sender: &OutputSender);
    }
}

pub struct Inputs<'a>(Pin<Box<dyn Stream<Item = Input> + 'a>>);

fn next_input(inputs: &mut Inputs) -> ffi::DoraInput {
    match block_on(inputs.0.next()) {
        Some(input) => ffi::DoraInput {
            end_of_input: false,
            id: input.id.into(),
            data: input.data,
        },
        None => ffi::DoraInput {
            end_of_input: true,
            id: String::new(),
            data: Vec::new(),
        },
    }
}

pub struct OutputSender<'a>(&'a DoraNode);

fn send_output(sender: &OutputSender, id: String, data: &[u8]) -> ffi::DoraResult {
    let error = match block_on(sender.0.send_output(&id.into(), data)) {
        Ok(()) => String::new(),
        Err(err) => format!("{err:?}"),
    };
    ffi::DoraResult { error }
}

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let node = DoraNode::init_from_env().await?;
    let input_stream = node.inputs().await?;
    let mut inputs = Inputs(Box::pin(input_stream));
    let outputs = OutputSender(&node);
    ffi::cxx_main(&mut inputs, &outputs);

    Ok(())
}
