use super::{OperatorEvent, StopReason};
use aligned_vec::{AVec, ConstAlign};
use dora_core::{
    adjust_shared_library_path,
    config::{DataId, NodeId, OperatorId},
    descriptor::source_is_url,
};
use dora_download::download_file;
use dora_node_api::{
    arrow_utils::{copy_array_into_sample, required_data_size},
    Event, MetadataParameters,
};
use dora_operator_api_types::{
    safer_ffi::closure::ArcDynFn1, DoraDropOperator, DoraInitOperator, DoraInitResult, DoraOnEvent,
    DoraResult, DoraStatus, Metadata, OnEventResult, Output, SendOutput,
};
use eyre::{bail, eyre, Context, Result};
use libloading::Symbol;
use std::{
    ffi::c_void,
    panic::{catch_unwind, AssertUnwindSafe},
    path::Path,
    sync::Arc,
};
use tokio::sync::{mpsc::Sender, oneshot};
use tracing::{field, span};

pub fn run(
    node_id: &NodeId,
    operator_id: &OperatorId,
    source: &str,
    events_tx: Sender<OperatorEvent>,
    incoming_events: flume::Receiver<Event>,
    init_done: oneshot::Sender<Result<()>>,
) -> eyre::Result<()> {
    let path = if source_is_url(source) {
        let target_path = adjust_shared_library_path(
            &Path::new("build")
                .join(node_id.to_string())
                .join(operator_id.to_string()),
        )?;
        // try to download the shared library
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()?;
        rt.block_on(download_file(source, &target_path))
            .wrap_err("failed to download shared library operator")?;
        target_path
    } else {
        adjust_shared_library_path(Path::new(source))?
    };

    let library = unsafe {
        libloading::Library::new(&path)
            .wrap_err_with(|| format!("failed to load shared library at `{}`", path.display()))?
    };

    let closure = AssertUnwindSafe(|| {
        let bindings = Bindings::init(&library).context("failed to init operator")?;

        let operator = SharedLibraryOperator {
            incoming_events,
            bindings,
            events_tx: events_tx.clone(),
        };

        operator.run(init_done)
    });
    match catch_unwind(closure) {
        Ok(Ok(reason)) => {
            let _ = events_tx.blocking_send(OperatorEvent::Finished { reason });
        }
        Ok(Err(err)) => {
            let _ = events_tx.blocking_send(OperatorEvent::Error(err));
        }
        Err(panic) => {
            let _ = events_tx.blocking_send(OperatorEvent::Panic(panic));
        }
    }

    Ok(())
}

struct SharedLibraryOperator<'lib> {
    incoming_events: flume::Receiver<Event>,
    events_tx: Sender<OperatorEvent>,

    bindings: Bindings<'lib>,
}

impl<'lib> SharedLibraryOperator<'lib> {
    fn run(self, init_done: oneshot::Sender<Result<()>>) -> eyre::Result<StopReason> {
        let operator_context = {
            let DoraInitResult {
                result,
                operator_context,
            } = unsafe { (self.bindings.init_operator.init_operator)() };
            let raw = match result.error {
                Some(error) => {
                    let _ = init_done.send(Err(eyre!(error.to_string())));
                    bail!("init_operator failed: {}", *error)
                }
                None => operator_context,
            };
            OperatorContext {
                raw,
                drop_fn: self.bindings.drop_operator.clone(),
            }
        };

        let _ = init_done.send(Ok(()));

        let send_output_closure = Arc::new(move |output: Output| {
            let Output {
                id: output_id,
                data_array,
                schema,
                metadata: Metadata {
                    open_telemetry_context,
                },
            } = output;
            let parameters = MetadataParameters {
                open_telemetry_context: open_telemetry_context.into(),
                ..Default::default()
            };

            let arrow_array = match unsafe { arrow::ffi::from_ffi(data_array, &schema) } {
                Ok(a) => a,
                Err(err) => return DoraResult::from_error(err.to_string()),
            };

            let total_len = required_data_size(&arrow_array);
            let mut sample: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, total_len);

            let type_info = copy_array_into_sample(&mut sample, &arrow_array);

            let event = OperatorEvent::Output {
                output_id: DataId::from(String::from(output_id)),
                type_info,
                parameters,
                data: Some(sample.into()),
            };

            let result = self
                .events_tx
                .blocking_send(event)
                .map_err(|_| eyre!("failed to send output to runtime"));

            match result {
                Ok(()) => DoraResult::SUCCESS,
                Err(_) => DoraResult::from_error("runtime process closed unexpectedly".into()),
            }
        });

        let reason = loop {
            #[allow(unused_mut)]
            let Ok(mut event) = self.incoming_events.recv() else {
                break StopReason::InputsClosed;
            };

            let span = span!(tracing::Level::TRACE, "on_event", input_id = field::Empty);
            let _ = span.enter();
            // Add metadata context if we have a tracer and
            // incoming input has some metadata.
            #[cfg(feature = "telemetry")]
            if let Event::Input {
                id: input_id,
                metadata,
                ..
            } = &mut event
            {
                use dora_tracing::telemetry::{deserialize_context, serialize_context};
                use tracing_opentelemetry::OpenTelemetrySpanExt;
                span.record("input_id", input_id.as_str());

                let cx = deserialize_context(&metadata.parameters.open_telemetry_context);
                span.set_parent(cx);
                let cx = span.context();
                let string_cx = serialize_context(&cx);
                metadata.parameters.open_telemetry_context = string_cx;
            }

            let mut operator_event = match event {
                Event::Stop => dora_operator_api_types::RawEvent {
                    input: None,
                    input_closed: None,
                    stop: true,
                    error: None,
                },
                Event::Input {
                    id: input_id,
                    metadata,
                    data,
                } => {
                    let (data_array, schema) = arrow::ffi::to_ffi(&data.to_data())?;

                    let operator_input = dora_operator_api_types::Input {
                        id: String::from(input_id).into(),
                        data_array: Some(data_array),
                        schema,
                        metadata: Metadata {
                            open_telemetry_context: metadata
                                .parameters
                                .open_telemetry_context
                                .into(),
                        },
                    };
                    dora_operator_api_types::RawEvent {
                        input: Some(Box::new(operator_input).into()),
                        input_closed: None,
                        stop: false,
                        error: None,
                    }
                }
                Event::InputClosed { id: input_id } => dora_operator_api_types::RawEvent {
                    input_closed: Some(input_id.to_string().into()),
                    input: None,
                    stop: false,
                    error: None,
                },
                Event::Reload { .. } => {
                    // Reloading shared lib operator is not supported. See: https://github.com/dora-rs/dora/pull/239#discussion_r1154313139
                    continue;
                }
                Event::Error(err) => dora_operator_api_types::RawEvent {
                    error: Some(err.into()),
                    input_closed: None,
                    input: None,
                    stop: false,
                },
                other => {
                    tracing::warn!("unexpected event: {other:?}");
                    continue;
                }
            };

            let send_output = SendOutput {
                send_output: ArcDynFn1::new(send_output_closure.clone()),
            };
            let OnEventResult {
                result: DoraResult { error },
                status,
            } = unsafe {
                (self.bindings.on_event.on_event)(
                    &mut operator_event,
                    &send_output,
                    operator_context.raw,
                )
            };
            match error {
                Some(error) => bail!("on_input failed: {}", *error),
                None => match status {
                    DoraStatus::Continue => {}
                    DoraStatus::Stop => break StopReason::ExplicitStop,
                    DoraStatus::StopAll => break StopReason::ExplicitStopAll,
                },
            }
        };
        Ok(reason)
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
    on_event: Symbol<'lib, DoraOnEvent>,
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
                on_event: library
                    .get(b"dora_on_event")
                    .wrap_err("failed to get `dora_on_event`")?,
            }
        };
        Ok(bindings)
    }
}
