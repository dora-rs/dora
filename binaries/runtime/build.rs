fn main() {
    // Re-emit `Py_3_N` cfgs from pyo3 into this crate so any future code that
    // wants to gate on Python ABI levels (`#[cfg(Py_3_11)]` etc.) can do so
    // without each contributor rediscovering this setup. See
    // apis/python/node/build.rs for context; #1833 hit this issue first.
    //
    // pyo3-build-config is an optional build-dependency tied to the `python`
    // feature: its own build script probes the Python interpreter and fails
    // on systems older than the workspace `abi3-py311` floor (e.g. Python
    // 3.10 on ubuntu-22.04), which broke non-Python `cargo build -p dora-cli`.
    #[cfg(feature = "python")]
    pyo3_build_config::use_pyo3_cfgs();
}
