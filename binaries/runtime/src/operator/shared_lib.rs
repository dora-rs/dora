use super::{IncomingEvent, OperatorEvent, StopReason, Tracer};
use dora_core::{
    adjust_shared_library_path,
    config::{DataId, NodeId, OperatorId},
    descriptor::source_is_url,
};
use dora_download::download_file;
use dora_node_api::MetadataParameters;
use dora_operator_api_types::{
    safer_ffi::closure::ArcDynFn1, DoraDropOperator, DoraInitOperator, DoraInitResult, DoraOnEvent,
    DoraResult, DoraStatus, Metadata, OnEventResult, Output, SendOutput,
};
use eyre::{bail, eyre, Context};
use libloading::Symbol;
use std::{
    borrow::Cow,
    ffi::c_void,
    panic::{catch_unwind, AssertUnwindSafe},
    path::Path,
    sync::Arc,
};
use tokio::sync::mpsc::Sender;

pub fn run(
    node_id: &NodeId,
    operator_id: &OperatorId,
    source: &str,
    events_tx: Sender<OperatorEvent>,
    incoming_events: flume::Receiver<IncomingEvent>,
    tracer: Tracer,
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

        operator.run(tracer)
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
    incoming_events: flume::Receiver<IncomingEvent>,
    events_tx: Sender<OperatorEvent>,

    bindings: Bindings<'lib>,
}

impl<'lib> SharedLibraryOperator<'lib> {
    fn run(self, tracer: Tracer) -> eyre::Result<StopReason> {
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
                id: output_id,
                data,
                metadata: Metadata {
                    open_telemetry_context,
                },
            } = output;
            let metadata = MetadataParameters {
                open_telemetry_context: Cow::Owned(open_telemetry_context.into()),
                ..Default::default()
            };

            let event = OperatorEvent::Output {
                output_id: DataId::from(String::from(output_id)),
                metadata,
                data: data.to_owned(),
            };

            let result = self
                .events_tx
                .blocking_send(event)
                .map_err(|_| eyre!("failed to send output to runtime"));

            let error = match result {
                Ok(()) => None,
                Err(_) => Some(String::from("runtime process closed unexpectedly").into()),
            };

            DoraResult { error }
        });

        let reason = loop {
            let Ok(mut event) = self.incoming_events.recv() else {
                break StopReason::InputsClosed
            };

            if let IncomingEvent::Input {
                input_id, metadata, ..
            } = &mut event
            {
                #[cfg(feature = "telemetry")]
                let (_child_cx, string_cx) = {
                    use dora_tracing::telemetry::{deserialize_context, serialize_context};
                    use opentelemetry::{
                        trace::{TraceContextExt, Tracer},
                        Context as OtelContext,
                    };

                    let span = tracer.start_with_context(
                        format!("{}", input_id),
                        &deserialize_context(&metadata.parameters.open_telemetry_context),
                    );
                    let child_cx = OtelContext::current_with_span(span);
                    let string_cx = serialize_context(&child_cx);
                    (child_cx, string_cx)
                };
                #[cfg(not(feature = "telemetry"))]
                let string_cx = {
                    let () = tracer;
                    let _ = input_id;
                    "".to_string()
                };
                metadata.parameters.open_telemetry_context = Cow::Owned(string_cx);
            }

            let operator_event = match event {
                IncomingEvent::Stop => dora_operator_api_types::RawEvent {
                    input: None,
                    input_closed: None,
                    stop: true,
                },
                IncomingEvent::Input {
                    input_id,
                    metadata,
                    data,
                } => {
                    let operator_input = dora_operator_api_types::Input {
                        id: String::from(input_id).into(),
                        data: data.unwrap_or_default().into(),
                        metadata: Metadata {
                            open_telemetry_context: metadata
                                .parameters
                                .open_telemetry_context
                                .into_owned()
                                .into(),
                        },
                    };
                    dora_operator_api_types::RawEvent {
                        input: Some(Box::new(operator_input).into()),
                        input_closed: None,
                        stop: false,
                    }
                }
                IncomingEvent::InputClosed { input_id } => dora_operator_api_types::RawEvent {
                    input_closed: Some(input_id.to_string().into()),
                    input: None,
                    stop: false,
                },
            };

            let send_output = SendOutput {
                send_output: ArcDynFn1::new(send_output_closure.clone()),
            };
            let OnEventResult {
                result: DoraResult { error },
                status,
            } = unsafe {
                (self.bindings.on_event.on_event)(
                    &operator_event,
                    &send_output,
                    operator_context.raw,
                )
            };
            match error {
                Some(error) => bail!("on_input failed: {}", String::from(error)),
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
