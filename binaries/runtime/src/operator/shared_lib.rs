use super::OperatorEvent;
use dora_node_api::{communication::Publisher, config::DataId};
use dora_operator_api_types::{
    safer_ffi::closure::ArcDynFn1, DoraDropOperator, DoraInitOperator, DoraInitResult, DoraOnInput,
    DoraResult, DoraStatus, Metadata, OnInputResult, Output, SendOutput,
};
use eyre::{bail, eyre, Context};
use flume::Receiver;
use libloading::Symbol;
use std::{
    collections::HashMap,
    ffi::c_void,
    ops::Deref,
    panic::{catch_unwind, AssertUnwindSafe},
    path::Path,
    sync::Arc,
    thread,
};
use tokio::sync::mpsc::Sender;

pub fn spawn(
    path: &Path,
    events_tx: Sender<OperatorEvent>,
    inputs: Receiver<dora_node_api::Input>,
    publishers: HashMap<DataId, Box<dyn Publisher>>,
) -> eyre::Result<()> {
    let file_name = path
        .file_name()
        .ok_or_else(|| eyre!("shared library path has no file name"))?
        .to_str()
        .ok_or_else(|| eyre!("shared library file name is not valid UTF8"))?;
    if file_name.starts_with("lib") {
        bail!("Shared library file name must not start with `lib`, prefix is added automatically");
    }
    if path.extension().is_some() {
        bail!("Shared library file name must have no extension, it is added automatically");
    }
    let path = path.with_file_name(libloading::library_filename(file_name));

    let library = unsafe {
        libloading::Library::new(&path)
            .wrap_err_with(|| format!("failed to load shared library at `{}`", path.display()))?
    };

    thread::spawn(move || {
        let closure = AssertUnwindSafe(|| {
            let bindings = Bindings::init(&library).context("failed to init operator")?;

            let operator = SharedLibraryOperator { inputs, bindings };

            operator.run(publishers)
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
    inputs: Receiver<dora_node_api::Input>,

    bindings: Bindings<'lib>,
}

impl<'lib> SharedLibraryOperator<'lib> {
    fn run(self, publishers: HashMap<DataId, Box<dyn Publisher>>) -> eyre::Result<()> {
        let operator_context = {
            let DoraInitResult {
                result,
                operator_context,
            } = unsafe { (self.bindings.init_operator.init_operator)() };
            let raw = match result.error {
                Some(error) => bail!("init_operator failed: {}", String::from(error)),
                None => operator_context,
            };
            OperatorContext {
                raw,
                drop_fn: self.bindings.drop_operator.clone(),
            }
        };

        let send_output_closure = Arc::new(move |output: Output| {
            let Output {
                id,
                data,
                metadata: Metadata {
                    open_telemetry_context,
                },
            } = output;
            let metadata = dora_node_api::Metadata {
                open_telemetry_context: String::from(open_telemetry_context).into(),
                ..Default::default()
            };

            let message = metadata
                .serialize()
                .context(format!("failed to serialize `{}` metadata", id.deref()))
                .map_err(|err| err.into());

            let result = message.and_then(|mut message| match publishers.get(id.deref()) {
                Some(publisher) => {
                    message.extend_from_slice(&data); // TODO avoid copy
                    publisher.publish(&message)
                }
                None => Err(eyre!(
                    "unexpected output {} (not defined in dataflow config)",
                    id.deref()
                )
                .into()),
            });

            let error = match result {
                Ok(()) => None,
                Err(_) => Some(String::from("runtime process closed unexpectedly").into()),
            };

            DoraResult { error }
        });

        while let Ok(input) = self.inputs.recv() {
            let operator_input = dora_operator_api_types::Input {
                data: input.data().into_owned().into(),
                id: String::from(input.id).into(),
                metadata: Metadata {
                    open_telemetry_context: String::new().into(),
                },
            };

            let send_output = SendOutput {
                send_output: ArcDynFn1::new(send_output_closure.clone()),
            };
            let OnInputResult {
                result: DoraResult { error },
                status,
            } = unsafe {
                (self.bindings.on_input.on_input)(
                    &operator_input,
                    &send_output,
                    operator_context.raw,
                )
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
        unsafe { (self.drop_fn.drop_operator)(self.raw) };
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
