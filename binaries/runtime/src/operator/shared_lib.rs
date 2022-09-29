use super::OperatorEvent;
use dora_node_api::{communication::Publisher, config::DataId};
use dora_operator_api_types::{
    safer_ffi::{
        self,
        closure::{ArcDynFn1, BoxDynFnMut0},
        slice::slice_raw,
    },
    DoraDropOperator, DoraInitOperator, DoraInitResult, DoraOnInput, DoraResult, DoraStatus,
    Metadata, OnInputResult, Output, OutputMetadata, PrepareOutput, PrepareOutputResult,
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
    ptr::NonNull,
    sync::{Arc, Mutex},
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

        let prepare_output_closure = Arc::new(move |output: OutputMetadata| {
            let OutputMetadata {
                id,
                data_len,
                metadata: Metadata {
                    open_telemetry_context,
                },
            } = output;
            let metadata = dora_node_api::Metadata {
                open_telemetry_context: String::from(open_telemetry_context).into(),
                ..Default::default()
            };

            let prepare = || {
                let serialized_metadata = metadata
                    .serialize()
                    .context(format!("failed to serialize `{}` metadata", id.deref()))?;
                let data_offset = serialized_metadata.len();

                match publishers.get(id.deref()) {
                    Some(publisher) => {
                        let mut sample = publisher
                            .prepare(data_offset + data_len)
                            .map_err(|err| eyre!(err))
                            .context("prepare failed")?;
                        sample.as_mut_slice()[..data_offset].copy_from_slice(&serialized_metadata);

                        let shared = Arc::new(Mutex::new(Some(sample)));

                        let shared_clone = shared.clone();
                        let data_mut = BoxDynFnMut0::new(Box::new(move || {
                            let mut sample = shared_clone.try_lock().unwrap();
                            let slice = &mut sample.as_mut().unwrap().as_mut_slice()[data_offset..];
                            slice_raw::from(safer_ffi::slice::Mut::from(slice))
                        }));

                        let send = BoxDynFnMut0::new(Box::new(move || {
                            let sample = shared.try_lock().unwrap().take();
                            let result = match sample {
                                Some(sample) => sample.publish(),
                                None => Err("send was called multiple times".into()),
                            };
                            let error = match result {
                                Ok(()) => None,
                                Err(err) => Some(err.to_string().into()),
                            };
                            DoraResult { error }
                        }));

                        Result::<_, eyre::Error>::Ok(Output { data_mut, send })
                    }
                    None => Err(eyre!(
                        "unexpected output {} (not defined in dataflow config)",
                        id.deref()
                    )
                    .into()),
                }
            };

            match prepare() {
                Ok(output) => PrepareOutputResult {
                    result: DoraResult { error: None },
                    output,
                },
                Err(_) => PrepareOutputResult {
                    result: DoraResult {
                        error: Some(String::from("runtime process closed unexpectedly").into()),
                    },
                    output: Output {
                        data_mut: BoxDynFnMut0::new(Box::new(|| slice_raw {
                            ptr: NonNull::new(1 as *mut u8).unwrap(),
                            len: 0,
                        })),
                        send: BoxDynFnMut0::new(Box::new(|| DoraResult {
                            error: Some("prepare output failed".to_owned().into()),
                        })),
                    },
                },
            }
        });

        while let Ok(input) = self.inputs.recv() {
            let operator_input = dora_operator_api_types::Input {
                data: input.data().into_owned().into(),
                id: String::from(input.id).into(),
                metadata: Metadata {
                    open_telemetry_context: String::new().into(),
                },
            };

            let send_output = PrepareOutput {
                prepare_output: ArcDynFn1::new(prepare_output_closure.clone()),
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
