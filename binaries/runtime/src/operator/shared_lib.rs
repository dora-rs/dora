use super::{OperatorEvent, OperatorInput};
use dora_operator_api_types::{
    safer_ffi::closure::RefDynFnMut1, DoraDropOperator, DoraInitOperator, DoraOnInput, DoraResult,
    DoraStatus, InitResult, Metadata, OnInputResult, Output, SendOutput,
};
use eyre::{bail, Context};
use libloading::Symbol;
use std::{
    ffi::c_void,
    panic::{catch_unwind, AssertUnwindSafe},
    path::Path,
    thread,
};
use tokio::sync::mpsc::{Receiver, Sender};

pub fn spawn(
    path: &Path,
    events_tx: Sender<OperatorEvent>,
    inputs: Receiver<OperatorInput>,
) -> eyre::Result<()> {
    let library = unsafe {
        libloading::Library::new(path)
            .wrap_err_with(|| format!("failed to load shared library at `{}`", path.display()))?
    };

    thread::spawn(move || {
        let closure = AssertUnwindSafe(|| {
            let bindings = Bindings::init(&library).context("failed to init operator")?;

            let operator = SharedLibraryOperator {
                events_tx: events_tx.clone(),
                inputs,
                bindings,
            };

            operator.run()
        });
        match catch_unwind(closure) {
            Ok(Ok(())) => {
                let _ = events_tx.blocking_send(OperatorEvent::Finished);
            }
            Ok(Err(err)) => {
                let _ = events_tx.blocking_send(OperatorEvent::Error(err));
            }
            Err(panic) => {
                let _ = events_tx.blocking_send(OperatorEvent::Panic(panic));
            }
        }
    });

    Ok(())
}

struct SharedLibraryOperator<'lib> {
    events_tx: Sender<OperatorEvent>,
    inputs: Receiver<OperatorInput>,

    bindings: Bindings<'lib>,
}

impl<'lib> SharedLibraryOperator<'lib> {
    fn run(mut self) -> eyre::Result<()> {
        let operator_context = {
            let InitResult {
                result,
                operator_context,
            } = unsafe { (self.bindings.init_operator)() };
            let raw = match result.error {
                Some(error) => bail!("init_operator failed: {}", String::from(error)),
                None => operator_context,
            };
            OperatorContext {
                raw,
                drop_fn: self.bindings.drop_operator.clone(),
            }
        };

        while let Some(input) = self.inputs.blocking_recv() {
            let operator_input = dora_operator_api_types::Input {
                id: String::from(input.id).into(),
                data: input.value.into(),
                metadata: Metadata {
                    open_telemetry_context: String::new().into(),
                },
            };

            let mut send_output_closure = |output: Output| {
                let id: String = output.id.into();
                let result = self.events_tx.blocking_send(OperatorEvent::Output {
                    id: id.into(),
                    value: output.data.into(),
                });

                let error = match result {
                    Ok(()) => None,
                    Err(_) => Some(String::from("runtime process closed unexpectedly").into()),
                };

                DoraResult { error }
            };
            let send_output = SendOutput(RefDynFnMut1::new(&mut send_output_closure));
            let OnInputResult {
                result: DoraResult { error },
                status,
            } = unsafe {
                (self.bindings.on_input)(&operator_input, send_output, operator_context.raw)
            };
            match error {
                Some(error) => bail!("on_input failed: {}", String::from(error)),
                None => match status {
                    DoraStatus::Continue => {}
                    DoraStatus::Stop => break,
                },
            }
        }
        Ok(())
    }
}

struct OperatorContext<'lib> {
    raw: *mut c_void,
    drop_fn: Symbol<'lib, DoraDropOperator>,
}

impl<'lib> Drop for OperatorContext<'lib> {
    fn drop(&mut self) {
        unsafe { (self.drop_fn)(self.raw) };
    }
}

struct Bindings<'lib> {
    init_operator: Symbol<'lib, DoraInitOperator>,
    drop_operator: Symbol<'lib, DoraDropOperator>,
    on_input: Symbol<'lib, DoraOnInput>,
}

impl<'lib> Bindings<'lib> {
    fn init(library: &'lib libloading::Library) -> Result<Self, eyre::Error> {
        let bindings = unsafe {
            Bindings {
                init_operator: library
                    .get(b"dora_init_operator")
                    .wrap_err("failed to get `dora_init_operator`")?,
                drop_operator: library
                    .get(b"dora_drop_operator")
                    .wrap_err("failed to get `dora_drop_operator`")?,
                on_input: library
                    .get(b"dora_on_input")
                    .wrap_err("failed to get `dora_on_input`")?,
            }
        };
        Ok(bindings)
    }
}
