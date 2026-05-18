fn main() {
    // Re-emit `Py_3_N` cfgs from pyo3 into this crate so we can gate code on
    // Python ABI levels (e.g. `#[cfg(Py_3_11)]`). Without this, downstream
    // code in this crate gets misleading "unused" warnings and the cfg is
    // never active — pyo3's build script only sets the cfg in pyo3's own
    // compilation unit. See apis/python/node/build.rs for the analogous
    // setup; #1833 (Python zero-copy send) hit this issue first.
    pyo3_build_config::use_pyo3_cfgs();
}
