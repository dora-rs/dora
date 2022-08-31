#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};
use ffi::SendOutputResult;

#[cxx::bridge]
#[allow(unsafe_op_in_unsafe_fn)]
mod ffi {
    struct OnInputResult {
        error: String,
        stop: bool,
    }

    struct SendOutputResult {
        error: String,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;

        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult;
    }

    unsafe extern "C++" {
        include!("cxx-dataflow-example-operator-rust-api/src/operator.h");
        type Operator;

        fn new_operator() -> UniquePtr<Operator>;

        fn on_input(
            op: Pin<&mut Operator>,
            id: &str,
            data: &[u8],
            output_sender: &mut OutputSender,
        ) -> OnInputResult;
    }
}

pub struct OutputSender<'a, 'b>(&'a mut DoraOutputSender<'b>);

fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> SendOutputResult {
    let error = sender
        .0
        .send(id.into(), data.to_owned())
        .err()
        .unwrap_or_default();
    SendOutputResult { error }
}

register_operator!(OperatorWrapper);

struct OperatorWrapper {
    operator: cxx::UniquePtr<ffi::Operator>,
}

impl Default for OperatorWrapper {
    fn default() -> Self {
        Self {
            operator: ffi::new_operator(),
        }
    }
}

impl DoraOperator for OperatorWrapper {
    fn on_input(
        &mut self,
        id: &str,
        data: &[u8],
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, std::string::String> {
        let operator = self.operator.as_mut().unwrap();
        let mut output_sender = OutputSender(output_sender);

        let result = ffi::on_input(operator, id, data, &mut output_sender);
        if result.error.is_empty() {
            Ok(match result.stop {
                false => DoraStatus::Continue,
                true => DoraStatus::Stop,
            })
        } else {
            Err(result.error)
        }
    }
}
