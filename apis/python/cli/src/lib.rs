use eyre::Context;
use pyo3::prelude::*;

/// Start a runtime for Operators
///
/// :rtype: None
#[pyfunction]
pub fn start_runtime() -> eyre::Result<()> {
    dora_runtime::main().wrap_err("Dora Runtime raised an error.")
}

/// Build a Dataflow, exactly the same way as `dora build` command line tool.
///
///
/// :type dataflow_path: str
/// :type uv: bool, optional
/// :type coordinator_addr: str, optional
/// :type coordinator_port: int, optional
/// :type force_local: bool, optional
/// :rtype: None
#[pyfunction]
#[pyo3(signature = (dataflow_path, uv=None, coordinator_addr=None, coordinator_port=None, force_local=false))]
pub fn build(
    dataflow_path: String,
    uv: Option<bool>,
    coordinator_addr: Option<String>,
    coordinator_port: Option<u16>,
    force_local: bool,
) -> eyre::Result<()> {
    dora_cli::build(dora_cli::BuildConfig {
        dataflow: dataflow_path,
        coordinator_addr: coordinator_addr
            .map(|addr| addr.parse())
            .transpose()
            .wrap_err("invalid coordinator_addr")?,
        coordinator_port,
        uv: uv.unwrap_or_default(),
        force_local,
        ..Default::default()
    })
}

/// Run a Dataflow, exactly the same way as `dora run` command line tool.
///
/// :type dataflow_path: str
/// :type uv: bool, optional
/// :type stop_after: float, optional
/// :rtype: None
#[pyfunction]
#[pyo3(signature = (dataflow_path, uv=None, stop_after=None))]
pub fn run(dataflow_path: String, uv: Option<bool>, stop_after: Option<f64>) -> eyre::Result<()> {
    use dora_cli::Executable;

    let stop_after_duration = stop_after.map(std::time::Duration::from_secs_f64);
    let mut run = dora_cli::RunCommand::new(dataflow_path);
    if let Some(uv) = uv {
        run.uv = uv;
    }
    if let Some(duration) = stop_after_duration {
        run.stop_after = Some(duration);
    }
    run.execute()
}

#[pymodule(name = "dora_cli")]
fn dora_cli_python(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(start_runtime, &m)?)?;
    m.add_function(wrap_pyfunction!(run, &m)?)?;
    m.add_function(wrap_pyfunction!(build, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;

    Ok(())
}
