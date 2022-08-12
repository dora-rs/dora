#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{self, register_operator, DoraOperator, DoraOutputSender, DoraStatus};

#[cxx::bridge]
#[allow(unsafe_op_in_unsafe_fn)]
mod ffi {
    struct OnInputResult {
        error: String,
        stop: bool,
    }

    extern "Rust" {
        type OutputSender<'a>;

        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> i32;
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

pub struct OutputSender<'a>(&'a mut DoraOutputSender);

fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> i32 {
    match sender.0.send(id, data) {
        Ok(()) => 0,
        Err(err_code) => err_code.try_into().unwrap(),
    }
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
    ) -> Result<DoraStatus, ()> {
        let operator = self.operator.as_mut().unwrap();
        let mut output_sender = OutputSender(output_sender);

        let result = ffi::on_input(operator, id, data, &mut output_sender);
        if result.error.is_empty() {
            Ok(match result.stop {
                false => DoraStatus::Continue,
                true => DoraStatus::Stop,
            })
        } else {
            Err(())
        }
    }
}
