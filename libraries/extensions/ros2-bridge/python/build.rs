fn main() {
    // Re-emit `Py_3_N` cfgs from pyo3 into this crate so any future code that
    // wants to gate on Python ABI levels (`#[cfg(Py_3_11)]` etc.) can do so
    // without each contributor rediscovering this setup. See
    // apis/python/node/build.rs for context; #1833 hit this issue first.
    pyo3_build_config::use_pyo3_cfgs();
}
