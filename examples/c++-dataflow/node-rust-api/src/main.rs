use dora_node_api::{self, communication::Receiver, DoraNode, Input};

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
        type Inputs;
        type OutputSender<'a>;

        fn next_input(inputs: &mut Inputs) -> DoraInput;
        fn send_output(output_sender: &mut OutputSender, id: String, data: &[u8]) -> DoraResult;
    }

    unsafe extern "C++" {
        include!("cxx-dataflow-example-node-rust-api/src/main.h");

        fn cxx_main(inputs: &mut Inputs, output_sender: &mut OutputSender);
    }
}

pub struct Inputs(Receiver<Input>);

fn next_input(inputs: &mut Inputs) -> ffi::DoraInput {
    match inputs.0.recv() {
        Ok(input) => ffi::DoraInput {
            end_of_input: false,
            id: input.id.into(),
            data: input.data,
        },
        Err(_) => ffi::DoraInput {
            end_of_input: true,
            id: String::new(),
            data: Vec::new(),
        },
    }
}

pub struct OutputSender<'a>(&'a mut DoraNode);

fn send_output(sender: &mut OutputSender, id: String, data: &[u8]) -> ffi::DoraResult {
    let error = match sender.0.send_output(&id.into(), data) {
        Ok(()) => String::new(),
        Err(err) => format!("{err:?}"),
    };
    ffi::DoraResult { error }
}

fn main() -> eyre::Result<()> {
    let mut node = DoraNode::init_from_env()?;
    let input_stream = node.inputs()?;
    let mut inputs = Inputs(input_stream);
    let mut outputs = OutputSender(&mut node);
    ffi::cxx_main(&mut inputs, &mut outputs);

    Ok(())
}
