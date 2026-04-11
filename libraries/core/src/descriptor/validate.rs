use crate::{
    adjust_shared_library_path,
    descriptor::{self, source_is_url},
    get_python_path,
};

use dora_message::{
    config::{Input, InputMapping, UserInputMapping},
    descriptor::{CoreNodeKind, DYNAMIC_SOURCE, OperatorSource, ResolvedNode, SHELL_SOURCE},
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, bail, eyre};
use std::{collections::BTreeMap, path::Path, process::Command, time::Duration};
use tracing::info;

use super::{Descriptor, DescriptorExt, resolve_path};
const VERSION: &str = env!("CARGO_PKG_VERSION");

pub fn check_dataflow(
    dataflow: &Descriptor,
    working_dir: &Path,
    remote_daemon_id: Option<&[&str]>,
    coordinator_is_remote: bool,
) -> eyre::Result<()> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;
    let mut has_python_operator = false;
    let mut errors: Vec<String> = Vec::new();

    // check that nodes and operators exist
    for node in nodes.values() {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(custom) => match &custom.source {
                dora_message::descriptor::NodeSource::Local => match custom.path.as_str() {
                    SHELL_SOURCE => (),
                    DYNAMIC_SOURCE => (),
                    source => {
                        if source_is_url(source) {
                            if let Err(err) = check_url(source) {
                                errors.push(format!("node `{}`: {err}", node.id));
                            }
                        } else if let Some(remote_daemon_id) = remote_daemon_id {
                            if let Some(deploy) = &node.deploy {
                                if let Some(machine) = &deploy.machine {
                                    if remote_daemon_id.contains(&machine.as_str())
                                        || coordinator_is_remote
                                    {
                                        info!("skipping path check for remote node `{}`", node.id);
                                    }
                                }
                            }
                        } else if custom.build.is_some() {
                            info!("skipping path check for node with build command");
                        } else if let Err(err) = resolve_path(source, working_dir) {
                            errors.push(format!("node `{}`: {err}", node.id));
                        };
                    }
                },
                dora_message::descriptor::NodeSource::GitBranch { .. } => {
                    info!("skipping check for node with git source");
                }
            },
            descriptor::CoreNodeKind::Runtime(runtime_node) => {
                for operator_definition in &runtime_node.operators {
                    match &operator_definition.config.source {
                        OperatorSource::SharedLibrary(path) => {
                            if source_is_url(path) {
                                if let Err(err) = check_url(path) {
                                    errors.push(format!(
                                        "node `{}`, operator `{}`: {err}",
                                        node.id, operator_definition.id,
                                    ));
                                }
                            } else if operator_definition.config.build.is_some() {
                                info!("skipping path check for operator with build command");
                            } else {
                                match adjust_shared_library_path(Path::new(&path)) {
                                    Ok(path) => {
                                        if !working_dir.join(&path).exists() {
                                            errors.push(format!(
                                                "node `{}`, operator `{}`: no shared library at `{}`",
                                                node.id,
                                                operator_definition.id,
                                                path.display()
                                            ));
                                        }
                                    }
                                    Err(err) => {
                                        errors.push(format!(
                                            "node `{}`, operator `{}`: {err}",
                                            node.id, operator_definition.id,
                                        ));
                                    }
                                }
                            }
                        }
                        OperatorSource::Python(python_source) => {
                            has_python_operator = true;
                            let path = &python_source.source;
                            if source_is_url(path) {
                                if let Err(err) = check_url(path) {
                                    errors.push(format!(
                                        "node `{}`, operator `{}`: {err}",
                                        node.id, operator_definition.id,
                                    ));
                                }
                            } else if !working_dir.join(path).exists() {
                                errors.push(format!(
                                    "node `{}`, operator `{}`: no Python library at `{path}`",
                                    node.id, operator_definition.id,
                                ));
                            }
                        }
                    }
                }
            }
        }
    }

    // check that all inputs mappings point to an existing output
    for node in nodes.values() {
        match &node.kind {
            descriptor::CoreNodeKind::Custom(custom_node) => {
                for (input_id, input) in &custom_node.run_config.inputs {
                    if let Err(err) = check_input(input, &nodes, &format!("{}/{input_id}", node.id))
                    {
                        errors.push(format!("{err}"));
                    }
                }
            }
            descriptor::CoreNodeKind::Runtime(runtime_node) => {
                for operator_definition in &runtime_node.operators {
                    for (input_id, input) in &operator_definition.config.inputs {
                        if let Err(err) = check_input(
                            input,
                            &nodes,
                            &format!("{}/{}/{input_id}", operator_definition.id, node.id),
                        ) {
                            errors.push(format!("{err}"));
                        }
                    }
                }
            }
        };
    }

    // Check that nodes can resolve `send_stdout_as`
    for node in nodes.values() {
        if let Err(err) = node.send_stdout_as() {
            errors.push(format!(
                "node `{}`: could not resolve `send_stdout_as` configuration: {err}",
                node.id
            ));
        }
    }

    if has_python_operator {
        if let Err(err) = check_python_runtime() {
            errors.push(format!("{err}"));
        }
    }

    if errors.is_empty() {
        Ok(())
    } else {
        let error_list = errors
            .iter()
            .map(|e| format!("  - {e}"))
            .collect::<Vec<_>>()
            .join("\n");
        bail!(
            "found {} validation error(s):\n{}",
            errors.len(),
            error_list
        );
    }
}

pub trait ResolvedNodeExt {
    fn send_stdout_as(&self) -> eyre::Result<Option<String>>;
}

impl ResolvedNodeExt for ResolvedNode {
    fn send_stdout_as(&self) -> eyre::Result<Option<String>> {
        match &self.kind {
            // TODO: Split stdout between operators
            CoreNodeKind::Runtime(n) => {
                let count = n
                    .operators
                    .iter()
                    .filter(|op| op.config.send_stdout_as.is_some())
                    .count();
                if count == 1 && n.operators.len() > 1 {
                    tracing::warn!(
                        "All stdout from all operators of a runtime are going to be sent in the selected `send_stdout_as` operator."
                    )
                } else if count > 1 {
                    return Err(eyre!(
                        "More than one `send_stdout_as` entries for a runtime node. Please only use one `send_stdout_as` per runtime."
                    ));
                }
                Ok(n.operators.iter().find_map(|op| {
                    op.config
                        .send_stdout_as
                        .clone()
                        .map(|stdout| format!("{}/{}", op.id, stdout))
                }))
            }
            CoreNodeKind::Custom(n) => Ok(n.send_stdout_as.clone()),
        }
    }
}

fn check_input(
    input: &Input,
    nodes: &BTreeMap<NodeId, super::ResolvedNode>,
    input_id_str: &str,
) -> Result<(), eyre::ErrReport> {
    match &input.mapping {
        InputMapping::Timer { interval: _ } => {}
        InputMapping::User(UserInputMapping { source, output }) => {
            let source_node = nodes.values().find(|n| &n.id == source).ok_or_else(|| {
                eyre!("source node `{source}` mapped to input `{input_id_str}` does not exist",)
            })?;
            match &source_node.kind {
                CoreNodeKind::Custom(custom_node) => {
                    if !custom_node.run_config.outputs.contains(output) {
                        bail!(
                            "output `{source}/{output}` mapped to \
                            input `{input_id_str}` does not exist",
                        );
                    }
                }
                CoreNodeKind::Runtime(runtime) => {
                    let (operator_id, output) = output.split_once('/').unwrap_or_default();
                    let operator_id = OperatorId::from(operator_id.to_owned());
                    let output = DataId::from(output.to_owned());

                    let operator = runtime
                        .operators
                        .iter()
                        .find(|o| o.id == operator_id)
                        .ok_or_else(|| {
                            eyre!(
                                "source operator `{source}/{operator_id}` used \
                                for input `{input_id_str}` does not exist",
                            )
                        })?;

                    if !operator.config.outputs.contains(&output) {
                        bail!(
                            "output `{source}/{operator_id}/{output}` mapped to \
                            input `{input_id_str}` does not exist",
                        );
                    }
                }
            }
        }
    };
    Ok(())
}

fn check_python_runtime() -> eyre::Result<()> {
    // Check if python dora-rs is installed and match cli version
    let reinstall_command =
        format!("Please reinstall it with: `pip install dora-rs=={VERSION} --force`");
    let mut command = Command::new(get_python_path().context("Could not get python binary")?);
    command.args([
        "-c",
        &format!(
            "
import dora;
assert dora.__version__=='{VERSION}',  'Python dora-rs should be {VERSION}, but current version is %s. {reinstall_command}' % (dora.__version__)
        "
        ),
    ]);
    let mut result = command
        .spawn()
        .wrap_err("Could not spawn python dora-rs command.")?;
    let status = result
        .wait()
        .wrap_err("Could not get exit status when checking python dora-rs")?;

    if !status.success() {
        bail!("Something went wrong with Python dora-rs. {reinstall_command}")
    }

    Ok(())
}

fn check_url(url: &str) -> eyre::Result<()> {
    let client = reqwest::blocking::Client::builder()
        .timeout(Duration::from_secs(5))
        .build()
        .wrap_err("failed to build HTTP client for URL validation")?;

    match client.head(url).send() {
        Ok(response) => {
            if response.status().is_success() {
                Ok(())
            } else if response.status() == reqwest::StatusCode::METHOD_NOT_ALLOWED {
                match client.get(url).send() {
                    Ok(get_response) if get_response.status().is_success() => Ok(()),
                    Ok(get_response) => eyre::bail!(
                        "URL `{}` is not reachable (status code: {})",
                        url,
                        get_response.status()
                    ),
                    Err(err) => eyre::bail!("Failed to reach URL `{}`: {}", url, err),
                }
            } else {
                eyre::bail!(
                    "URL `{}` is not reachable (status code: {})",
                    url,
                    response.status()
                )
            }
        }
        Err(err) => eyre::bail!("Failed to reach URL `{}`: {}", url, err),
    }
}

#[cfg(test)]
mod tests {
    use super::check_url;
    use std::io::{Read, Write};
    use std::net::{TcpListener, TcpStream};
    use std::thread;

    fn read_request_method(stream: &mut TcpStream) -> String {
        let mut buffer = [0_u8; 2048];
        let bytes_read = stream.read(&mut buffer).expect("failed to read request");
        let request = std::str::from_utf8(&buffer[..bytes_read]).expect("invalid UTF-8 in request");
        request
            .lines()
            .next()
            .and_then(|line| line.split_whitespace().next())
            .unwrap_or_default()
            .to_string()
    }

    #[test]
    fn check_url_accepts_success_status() {
        let listener = TcpListener::bind("127.0.0.1:0").expect("failed to bind test server");
        let addr = listener.local_addr().expect("failed to get local addr");

        let handle = thread::spawn(move || {
            let (mut stream, _) = listener.accept().expect("failed to accept connection");
            let _ = read_request_method(&mut stream);
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
                .expect("failed to write response");
        });

        check_url(&format!("http://{addr}/ok")).expect("URL should be reachable");
        handle.join().expect("server thread panicked");
    }

    #[test]
    fn check_url_rejects_non_success_status() {
        let listener = TcpListener::bind("127.0.0.1:0").expect("failed to bind test server");
        let addr = listener.local_addr().expect("failed to get local addr");

        let handle = thread::spawn(move || {
            let (mut stream, _) = listener.accept().expect("failed to accept connection");
            let _ = read_request_method(&mut stream);
            stream
                .write_all(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
                .expect("failed to write response");
        });

        let err = check_url(&format!("http://{addr}/missing")).expect_err("URL should be rejected");
        assert!(err.to_string().contains("status code: 404"));
        handle.join().expect("server thread panicked");
    }

    #[test]
    fn check_url_falls_back_to_get_when_head_not_allowed() {
        let listener = TcpListener::bind("127.0.0.1:0").expect("failed to bind test server");
        let addr = listener.local_addr().expect("failed to get local addr");

        let handle = thread::spawn(move || {
            for _ in 0..2 {
                let (mut stream, _) = listener.accept().expect("failed to accept connection");
                let method = read_request_method(&mut stream);
                if method == "HEAD" {
                    stream
                        .write_all(b"HTTP/1.1 405 Method Not Allowed\r\nConnection: close\r\nContent-Length: 0\r\n\r\n")
                        .expect("failed to write response");
                } else {
                    stream
                        .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
                        .expect("failed to write response");
                }
            }
        });

        check_url(&format!("http://{addr}/head-not-allowed"))
            .expect("GET fallback should mark URL as reachable");
        handle.join().expect("server thread panicked");
    }
}
