fn main() {
    // Re-emit `Py_3_N` cfgs from pyo3 into this crate so any future code that
    // wants to gate on Python ABI levels (`#[cfg(Py_3_11)]` etc.) can do so
    // without each contributor rediscovering this setup. See
    // apis/python/node/build.rs for context; #1833 hit this issue first.
    //
    // `pyo3` is an optional dependency on this crate (`python` feature), but
    // `pyo3-build-config::use_pyo3_cfgs()` is a no-op when there's no Python
    // discoverable, so it's safe to call unconditionally.
    pyo3_build_config::use_pyo3_cfgs();
}
