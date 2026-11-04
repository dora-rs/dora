#![warn(unsafe_op_in_unsafe_fn)]
// The operator itself is `cfg(not(test))`: a test binary cannot link the C++
// operator functions (`new_operator`, `on_input`, …), which only exist in a
// real operator library. The input conversion below stays testable; the
// bridge stays unconditional (cxx rejects `cfg(test)` there), so its unused
// C++ functions are allowed in test builds.
#![cfg_attr(test, allow(dead_code))]

use dora_operator_api::{DoraArray, DoraOutputSender, IntoArrow};
#[cfg(not(test))]
use dora_operator_api::{DoraOperator, DoraStatus, Event, register_operator};
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

        /// Called for each input. `data` is the payload's bytes; it is
        /// empty for a metadata-only input such as a timer tick, so
        /// check its length before indexing.
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

        /// Called when an input's Arrow data fails to deserialize
        /// (`Event::InputParseError { id, error }`). The daemon emits
        /// this when `arrow::ffi::from_ffi` returns `Err` for a
        /// received input. Without this callback the error was silently
        /// swallowed by the catch-all `_ => Continue` arm, leaving the
        /// operator with no way to log, surface as health, or reroute
        /// malformed inputs.
        fn on_input_parse_error(
            op: Pin<&mut Operator>,
            id: &str,
            error: &str,
            output_sender: &mut OutputSender,
        ) -> DoraOnInputResult;
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

#[cfg(not(test))]
register_operator!(OperatorWrapper);

#[cfg(not(test))]
struct OperatorWrapper {
    operator: cxx::UniquePtr<ffi::Operator>,
}

#[cfg(not(test))]
impl Default for OperatorWrapper {
    fn default() -> Self {
        Self {
            operator: ffi::new_operator(),
        }
    }
}

#[cfg(not(test))]
impl OperatorWrapper {
    /// The C++ operator instance, or an error if `new_operator()` returned
    /// null. Acquired lazily so events without an operator-side callback do
    /// not depend on it.
    fn operator(&mut self) -> Result<std::pin::Pin<&mut ffi::Operator>, std::string::String> {
        self.operator
            .as_mut()
            .ok_or_else(|| "C++ new_operator() returned a null operator".to_string())
    }
}

#[cfg(not(test))]
impl DoraOperator for OperatorWrapper {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, std::string::String> {
        let mut output_sender = OutputSender(output_sender);
        let result = match event {
            Event::Input {
                id,
                metadata: _,
                data,
            } => {
                let data = input_bytes(data)?;
                ffi::on_input(self.operator()?, id, data, &mut output_sender)
            }
            Event::InputClosed { id } => {
                ffi::on_input_closed(self.operator()?, id, &mut output_sender)
            }
            Event::Stop => ffi::on_stop(self.operator()?, &mut output_sender),
            Event::InputParseError { id, error } => {
                ffi::on_input_parse_error(self.operator()?, id, error, &mut output_sender)
            }
            // Other events (NodeFailed, Reload, Error, …) currently have no
            // operator-side callback, so they return `Continue` without
            // requiring the operator. Operators that need to react to them
            // should subscribe via the node API instead.
            _ => return Ok(DoraStatus::Continue),
        };
        finish(result)
    }
}

/// The bytes of an input payload, as handed to the C++ `on_input`.
///
/// Metadata-only inputs -- timer ticks, or a sender passing `()` -- arrive as
/// an empty `Null` array, which is not a `UInt8` array: any empty payload is
/// handed to C++ as an empty slice, as the C/C++ node APIs already do for
/// `Null`.
fn input_bytes(data: &DoraArray) -> Result<&[u8], String> {
    if data.is_empty() {
        Ok(&[])
    } else {
        data.try_into()
            .map_err(|err| format!("expected byte array: {err}"))
    }
}

/// Translate a C++ callback result into a [`DoraStatus`], surfacing any
/// non-empty error string to the operator runtime.
#[cfg(not(test))]
fn finish(result: ffi::DoraOnInputResult) -> Result<DoraStatus, std::string::String> {
    if result.error.is_empty() {
        Ok(if result.stop {
            DoraStatus::Stop
        } else {
            DoraStatus::Continue
        })
    } else {
        Err(result.error)
    }
}

#[cfg(test)]
mod tests {
    use super::input_bytes;
    use dora_operator_api::IntoArrow;

    #[test]
    fn timer_tick_input_is_an_empty_slice() {
        // A timer tick or a `()` send arrives as an empty `Null` array.
        assert_eq!(input_bytes(&().into_arrow()), Ok(&[][..]));
    }

    #[test]
    fn byte_inputs_pass_through() {
        assert_eq!(input_bytes(&Vec::<u8>::new().into_arrow()), Ok(&[][..]));
        assert_eq!(
            input_bytes(&vec![1u8, 2, 3].into_arrow()),
            Ok(&[1u8, 2, 3][..])
        );
    }

    #[test]
    fn non_byte_input_is_an_error() {
        let err = input_bytes(&vec![1.0f32].into_arrow()).unwrap_err();
        assert!(err.starts_with("expected byte array"), "{err}");
    }
}
