use crate::{graph::read_descriptor, zenoh_control_session};
use dora_core::{
    adjust_shared_library_path,
    config::{InputMapping, UserInputMapping},
    descriptor::{self, source_is_url, CoreNodeKind, OperatorSource},
    topics::ZENOH_CONTROL_LIST,
};
use eyre::{bail, eyre, Context};
use std::{env::consts::EXE_EXTENSION, io::Write, path::Path};
use sysinfo::SystemExt;
use termcolor::{Color, ColorChoice, ColorSpec, WriteColor};
use zenoh::{prelude::Receiver, sync::ZFuture};

pub fn check_environment() -> eyre::Result<()> {
    let mut error_occured = false;

    let color_choice = if atty::is(atty::Stream::Stdout) {
        ColorChoice::Auto
    } else {
        ColorChoice::Never
    };
    let mut stdout = termcolor::StandardStream::stdout(color_choice);

    // check whether coordinator is running

    write!(stdout, "Dora Coordinator: ")?;
    if coordinator_running()? {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
        writeln!(stdout, "ok")?;
    } else {
        let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
        writeln!(stdout, "not running")?;
        error_occured = true;
    }
    let _ = stdout.reset();

    // check whether roudi is running
    write!(stdout, "Iceoryx Daemon: ")?;
    let system = sysinfo::System::new_all();
    match system.processes_by_exact_name("iox-roudi").next() {
        Some(_) => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Green)));
            writeln!(stdout, "ok")?;
        }
        None => {
            let _ = stdout.set_color(ColorSpec::new().set_fg(Some(Color::Red)));
            writeln!(stdout, "not running")?;
            error_occured = true;
        }
    }
    writeln!(stdout)?;

    if error_occured {
        bail!("Environment check failed.");
    }

    Ok(())
}

pub fn coordinator_running() -> Result<bool, eyre::ErrReport> {
    let mut control_session = None;
    let reply_receiver = zenoh_control_session(&mut control_session)?
        .get(ZENOH_CONTROL_LIST)
        .wait()
        .map_err(|err| eyre!(err))
        .wrap_err("failed to create publisher for list message")?;
    Ok(reply_receiver.recv().is_ok())
}

pub fn check_dataflow(dataflow_path: &Path, runtime: Option<&Path>) -> eyre::Result<()> {
    let descriptor = read_descriptor(dataflow_path).wrap_err_with(|| {
        format!(
            "failed to read dataflow descriptor at {}",
            dataflow_path.display()
        )
    })?;
    let base = dataflow_path
        .canonicalize()
        .unwrap()
        .parent()
        .unwrap()
        .to_owned();

    let nodes = descriptor.resolve_aliases();

    if nodes
        .iter()
        .any(|n| matches!(n.kind, CoreNodeKind::Runtime(_)))
    {
        let runtime = runtime
            .unwrap_or_else(|| Path::new("dora-runtime"))
            .with_extension(EXE_EXTENSION);
        if !runtime.is_file() {
            bail!(
                "There is no runtime at {}, or it is not a file",
                runtime.display()
            );
        }
    }

    // check that nodes and operators exist
    for node in &nodes {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(node) => {
                let path = if source_is_url(&node.source) {
                    todo!("check URL");
                } else {
                    let raw = Path::new(&node.source);
                    if raw.extension().is_none() {
                        raw.with_extension(EXE_EXTENSION)
                    } else {
                        raw.to_owned()
                    }
                };
                base.join(&path)
                    .canonicalize()
                    .wrap_err_with(|| format!("no node exists at `{}`", path.display()))?;
            }
            descriptor::CoreNodeKind::Runtime(node) => {
                for operator_definition in &node.operators {
                    match &operator_definition.config.source {
                        OperatorSource::SharedLibrary(path) => {
                            if source_is_url(path) {
                                todo!("check URL");
                            } else {
                                let path = adjust_shared_library_path(Path::new(&path))?;
                                if !base.join(&path).exists() {
                                    bail!("no shared library at `{}`", path.display());
                                }
                            }
                        }
                        OperatorSource::Python(path) => {
                            if source_is_url(path) {
                                todo!("check URL");
                            } else {
                                if !base.join(&path).exists() {
                                    bail!("no Python library at `{path}`");
                                }
                            }
                        }
                        OperatorSource::Wasm(path) => {
                            if source_is_url(path) {
                                todo!("check URL");
                            } else {
                                if !base.join(&path).exists() {
                                    bail!("no WASM library at `{path}`");
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // check that all inputs mappings point to an existing output
    for node in &nodes {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(custom_node) => {
                for (input_id, mapping) in &custom_node.run_config.inputs {
                    check_input(mapping, &nodes, &format!("{}/{input_id}", node.id))?;
                }
            }
            descriptor::CoreNodeKind::Runtime(runtime_node) => {
                for operator_definition in &runtime_node.operators {
                    for (input_id, mapping) in &operator_definition.config.inputs {
                        check_input(
                            mapping,
                            &nodes,
                            &format!("{}/{}/{input_id}", operator_definition.id, node.id),
                        )?;
                    }
                }
            }
        };
    }

    check_environment()?;

    Ok(())
}

fn check_input(
    mapping: &InputMapping,
    nodes: &[dora_core::descriptor::ResolvedNode],
    input_id_str: &str,
) -> Result<(), eyre::ErrReport> {
    match mapping {
        InputMapping::Timer { interval: _ } => {}
        InputMapping::User(UserInputMapping {
            source,
            operator,
            output,
        }) => {
            let source_node = nodes.iter().find(|n| &n.id == source).ok_or_else(|| {
                eyre!("source node `{source}` mapped to input `{input_id_str}` does not exist",)
            })?;
            if let Some(operator_id) = operator {
                let operator = match &source_node.kind {
                    CoreNodeKind::Runtime(runtime) => {
                        let operator = runtime.operators.iter().find(|o| &o.id == operator_id);
                        operator.ok_or_else(|| {
                            eyre!(
                                "source operator `{source}/{operator_id}` used \
                                for input `{input_id_str}` does not exist",
                            )
                        })?
                    }
                    CoreNodeKind::Custom(_) => {
                        bail!(
                            "input `{input_id_str}` references operator \
                            `{source}/{operator_id}`, but `{source}` is a \
                            custom node",
                        );
                    }
                };

                if !operator.config.outputs.contains(output) {
                    bail!(
                        "output `{source}/{operator_id}/{output}` mapped to \
                        input `{input_id_str}` does not exist",
                    );
                }
            } else {
                match &source_node.kind {
                    CoreNodeKind::Runtime(_) => bail!(
                        "input `{input_id_str}` references output \
                        `{source}/{output}`, but `{source}` is a \
                        runtime node",
                    ),
                    CoreNodeKind::Custom(custom_node) => {
                        if !custom_node.run_config.outputs.contains(output) {
                            bail!(
                                "output `{source}/{output}` mapped to \
                                input `{input_id_str}` does not exist",
                            );
                        }
                    }
                }
            }
        }
    };
    Ok(())
}
