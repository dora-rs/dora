use dora_node_api::{self, DoraNode, Input, Receiver};

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
        Ok(input) => {
            let id = input.id.clone().into();
            let data = input.data();
            ffi::DoraInput {
                end_of_input: false,
                id,
                data: data.into_owned(),
            }
        }
        Err(_) => ffi::DoraInput {
            end_of_input: true,
            id: String::new(),
            data: Vec::new(),
        },
    }
}

pub struct OutputSender<'a>(&'a mut DoraNode);

fn send_output(sender: &mut OutputSender, id: String, data: &[u8]) -> ffi::DoraResult {
    let output = sender
        .0
        .prepare_output(&id.into(), &Default::default(), data.len());
    let result = output.and_then(|mut output| {
        output.data_mut().copy_from_slice(data);
        output.send()
    });
    let error = match result {
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
