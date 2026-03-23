#![cfg(not(test))]
#![warn(unsafe_op_in_unsafe_fn)]

use adora_operator_api::{
    self, AdoraOperator, AdoraOutputSender, AdoraStatus, Event, IntoArrow, register_operator,
};
use ffi::AdoraSendOutputResult;

#[cxx::bridge]
#[allow(unsafe_op_in_unsafe_fn)]
mod ffi {
    struct AdoraOnInputResult {
        error: String,
        stop: bool,
    }

    struct AdoraSendOutputResult {
        error: String,
    }

    extern "Rust" {
        type OutputSender<'a, 'b>;

        fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> AdoraSendOutputResult;
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
        ) -> AdoraOnInputResult;
    }
}

pub struct OutputSender<'a, 'b>(&'a mut AdoraOutputSender<'b>);

fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> AdoraSendOutputResult {
    let error = sender
        .0
        .send(id, data.to_owned().into_arrow())
        .err()
        .unwrap_or_default();
    AdoraSendOutputResult { error }
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

impl AdoraOperator for OperatorWrapper {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut AdoraOutputSender,
    ) -> Result<AdoraStatus, std::string::String> {
        match event {
            Event::Input { id, metadata: _, data } => {
                let operator = self.operator.as_mut().unwrap();
                let mut output_sender = OutputSender(output_sender);
                let data: &[u8] = data
                    .try_into()
                    .map_err(|err| format!("expected byte array: {err}"))?;

                let result = ffi::on_input(operator, id, data, &mut output_sender);
                if result.error.is_empty() {
                    Ok(match result.stop {
                        false => AdoraStatus::Continue,
                        true => AdoraStatus::Stop,
                    })
                } else {
                    Err(result.error)
                }
            }
            _ => {
                // ignore other events for now
                Ok(AdoraStatus::Continue)
            }
        }
    }
}
