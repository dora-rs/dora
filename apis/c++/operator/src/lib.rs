#![cfg(not(test))]
#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{
    self, register_operator, DoraOperator, DoraOutputSender, DoraStatus, Event,
};
use ffi::DoraSendOutputResult;

#[cxx::bridge]
#[allow(unsafe_op_in_unsafe_fn)]
mod ffi {
    struct DoraOnInputResult {
        error: String,
        stop: bool,
    }

    struct DoraSendOutputResult {
        error: String,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;

        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> DoraSendOutputResult;
    }

    unsafe extern "C++" {
        include!("operator.h");

        type Operator;

        fn new_operator() -> UniquePtr<Operator>;

        fn on_input(
            op: Pin<&mut Operator>,
            id: &str,
            data: &[u8],
            output_sender: &mut OutputSender,
        ) -> DoraOnInputResult;
    }
}

pub struct OutputSender<'a, 'b>(&'a mut DoraOutputSender<'b>);

fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> DoraSendOutputResult {
    let error = sender
        .0
        .send(id.into(), data.to_owned())
        .err()
        .unwrap_or_default();
    DoraSendOutputResult { error }
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
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, std::string::String> {
        match event {
            Event::Input { id, data } => {
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
            _ => {
                // ignore other events for now
                Ok(DoraStatus::Continue)
            }
        }
    }
}
