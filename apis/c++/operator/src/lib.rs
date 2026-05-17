#![cfg(not(test))]
#![warn(unsafe_op_in_unsafe_fn)]

use dora_operator_api::{
    self, DoraOperator, DoraOutputSender, DoraStatus, Event, IntoArrow, register_operator,
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

        /// Called when an upstream input stream closes (the daemon
        /// delivers `Event::InputClosed { id }`). Previously these
        /// events were silently dropped on the C++ operator side via
        /// the catch-all `_ => Continue` arm. The `output_sender` is
        /// the same per-event sender supplied to `on_input`, so the
        /// operator can emit a final/status output (e.g. a "drain
        /// complete" marker for a downstream consumer) in response to
        /// the input close.
        fn on_input_closed(
            op: Pin<&mut Operator>,
            id: &str,
            output_sender: &mut OutputSender,
        ) -> DoraOnInputResult;

        /// Called on graceful shutdown (the daemon delivers
        /// `Event::Stop` — a unit variant on `dora_operator_api::Event`,
        /// distinct from `dora_node_api::Event::Stop(StopCause)` which
        /// carries a payload). Previously silently dropped like
        /// `InputClosed`. The `output_sender` is provided so the
        /// operator can emit a final output (e.g. flush buffered
        /// state, send a "shutdown summary") before returning.
        fn on_stop(op: Pin<&mut Operator>, output_sender: &mut OutputSender) -> DoraOnInputResult;
    }
}

pub struct OutputSender<'a, 'b>(&'a mut DoraOutputSender<'b>);

fn send_output(sender: &mut OutputSender, id: &str, data: &[u8]) -> DoraSendOutputResult {
    let error = sender
        .0
        .send(id, data.to_owned().into_arrow())
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
            Event::Input {
                id,
                metadata: _,
                data,
            } => {
                let operator = self.operator.as_mut().unwrap();
                let mut output_sender = OutputSender(output_sender);
                let data: &[u8] = data
                    .try_into()
                    .map_err(|err| format!("expected byte array: {err}"))?;

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
            Event::InputClosed { id } => {
                let operator = self.operator.as_mut().unwrap();
                let mut output_sender = OutputSender(output_sender);
                let result = ffi::on_input_closed(operator, id, &mut output_sender);
                if result.error.is_empty() {
                    Ok(match result.stop {
                        false => DoraStatus::Continue,
                        true => DoraStatus::Stop,
                    })
                } else {
                    Err(result.error)
                }
            }
            Event::Stop => {
                let operator = self.operator.as_mut().unwrap();
                let mut output_sender = OutputSender(output_sender);
                let result = ffi::on_stop(operator, &mut output_sender);
                if result.error.is_empty() {
                    Ok(match result.stop {
                        false => DoraStatus::Continue,
                        true => DoraStatus::Stop,
                    })
                } else {
                    Err(result.error)
                }
            }
            // Other events (NodeFailed, Reload, Error, …) currently
            // have no operator-side callback. Operators that need to
            // react to them should subscribe via the node API instead.
            _ => Ok(DoraStatus::Continue),
        }
    }
}
