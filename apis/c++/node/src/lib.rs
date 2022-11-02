use dora_node_api::{self, Input, Receiver};

#[cxx::bridge]
mod ffi {
    struct DoraNode {
        inputs: Box<Inputs>,
        send_output: Box<OutputSender>,
    }

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
        type OutputSender;

        fn init_dora_node() -> Result<DoraNode>;
        fn free_dora_node(node: DoraNode);
        fn next_input(inputs: &mut Box<Inputs>) -> DoraInput;
        fn send_output(
            output_sender: &mut Box<OutputSender>,
            id: String,
            data: &[u8],
        ) -> DoraResult;
    }
}

fn init_dora_node() -> eyre::Result<ffi::DoraNode> {
    let mut node = dora_node_api::DoraNode::init_from_env()?;
    let input_stream = node.inputs()?;
    let inputs = Inputs(input_stream);
    let send_output = OutputSender(node);

    Ok(ffi::DoraNode {
        inputs: Box::new(inputs),
        send_output: Box::new(send_output),
    })
}

fn free_dora_node(node: ffi::DoraNode) {
    let _ = node;
}

pub struct Inputs(Receiver<Input>);

fn next_input(inputs: &mut Box<Inputs>) -> ffi::DoraInput {
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

pub struct OutputSender(dora_node_api::DoraNode);

fn send_output(sender: &mut Box<OutputSender>, id: String, data: &[u8]) -> ffi::DoraResult {
    let result = sender
        .0
        .send_output(&id.into(), Default::default(), data.len(), |out| {
            out.copy_from_slice(data)
        });
    let error = match result {
        Ok(()) => String::new(),
        Err(err) => format!("{err:?}"),
    };
    ffi::DoraResult { error }
}
