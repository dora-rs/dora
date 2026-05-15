fn main() {
    // Re-emit `Py_3_N` cfgs from pyo3 into this crate so we can gate code on
    // Python ABI levels (e.g. `#[cfg(Py_3_11)]` for the stable buffer-protocol
    // slots used by `send_output_raw`).
    pyo3_build_config::use_pyo3_cfgs();
    pyo3_build_config::add_extension_module_link_args();
}
