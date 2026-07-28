fn main() {
    // Re-emit `Py_3_N` cfgs from pyo3 into this crate so any future code that
    // wants to gate on Python ABI levels (`#[cfg(Py_3_11)]` etc.) can do so
    // without each contributor rediscovering this setup. See
    // apis/python/node/build.rs for context; #1833 hit this issue first.
    //
    // Unconditional here: this crate *is* the Python runtime backend and always
    // links pyo3. In the old monolithic `dora-runtime` this was gated behind the
    // `python` feature, because that crate was also linked into `dora-cli` on
    // systems below the workspace `abi3-py311` floor (e.g. Python 3.10 on
    // ubuntu-22.04), where pyo3-build-config's interpreter probe fails.
    pyo3_build_config::use_pyo3_cfgs();
}
