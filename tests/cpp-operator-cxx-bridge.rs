//! Smoke test: exercise the cxx-bridge `on_input_parse_error` dispatch path.
//!
//! Loads the compiled `operator_rust_api` shared library via `libloading`,
//! calls `dora_on_event` with a malformed Arrow FFI array, and verifies the
//! C++ callback was invoked and returned cleanly.
//!
//! The test is **skipped** (not failed) when the shared library is absent so
//! it does not break CI configurations that omit the C++ toolchain.  It is
//! intended to run as part of the nightly / full build after
//! `cargo run --example cxx-dataflow` has produced the artifacts.
//!
//! What is verified
//! ----------------
//! * `dora_on_event` / `dora_init_operator` / `dora_drop_operator` are
//!   exported and loadable (catches link-time symbol regressions).
//! * Passing a *released* `FFI_ArrowArray` (all-zero struct → `release = None`)
//!   causes `arrow::ffi::from_ffi` to return `Err`, which
//!   `raw::dora_on_event` converts to `Event::InputParseError` and dispatches
//!   to `OperatorWrapper::on_event` → `ffi::on_input_parse_error` (the
//!   cxx-bridge call site added in #1879).
//! * The C++ stub returns `{error: "", stop: false}`, so the final
//!   `OnEventResult` is `{ result: SUCCESS, status: Continue }`.

use std::{mem, path::PathBuf, sync::Arc};

use arrow_array::{Array, Int32Array};
use dora_operator_api_types::{
    DoraInitResult, DoraResult, DoraStatus, Input, Metadata, OnEventResult, RawEvent, SendOutput,
    arrow::ffi::FFI_ArrowArray, safer_ffi::closure::ArcDynFn1,
};

fn shared_lib_path() -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let build_dir = manifest.join("examples/c++-dataflow/build");
    build_dir.join(format!(
        "{}operator_rust_api{}",
        std::env::consts::DLL_PREFIX,
        std::env::consts::DLL_SUFFIX,
    ))
}

fn noop_send_output() -> SendOutput {
    use dora_operator_api_types::Output;
    SendOutput {
        send_output: ArcDynFn1::new(Arc::new(|_: Output| DoraResult::SUCCESS)),
    }
}

#[test]
fn cxx_bridge_dispatches_input_parse_error() {
    let lib_path = shared_lib_path();
    if !lib_path.exists() {
        eprintln!("SKIP: {lib_path:?} not found — run `cargo run --example cxx-dataflow` first");
        return;
    }

    // SAFETY: trusted shared library produced by the cxx-dataflow example build.
    let lib = unsafe { libloading::Library::new(&lib_path) }
        .unwrap_or_else(|e| panic!("failed to load {lib_path:?}: {e}"));

    type InitFn = unsafe extern "C" fn() -> DoraInitResult;
    type OnEventFn =
        unsafe extern "C" fn(&mut RawEvent, &SendOutput, *mut std::ffi::c_void) -> OnEventResult;
    type DropFn = unsafe extern "C" fn(*mut std::ffi::c_void) -> DoraResult;

    // SAFETY: symbol names and types match the register_operator!-generated exports.
    let init: libloading::Symbol<InitFn> =
        unsafe { lib.get(b"dora_init_operator\0") }.expect("dora_init_operator not found");
    let on_event_sym: libloading::Symbol<OnEventFn> =
        unsafe { lib.get(b"dora_on_event\0") }.expect("dora_on_event not found");
    let drop_op: libloading::Symbol<DropFn> =
        unsafe { lib.get(b"dora_drop_operator\0") }.expect("dora_drop_operator not found");

    let DoraInitResult {
        result: init_result,
        operator_context,
    } = unsafe { init() };
    assert!(
        init_result.error.is_none(),
        "dora_init_operator failed: {:?}",
        init_result.error
    );

    // Construct a valid schema from a real Int32Array, then substitute a
    // zeroed (released) FFI_ArrowArray.  arrow::ffi::from_ffi returns Err
    // immediately when release == None, without reading the schema, so the
    // schema only needs to be valid in memory (not semantically matched).
    let arr = Int32Array::from(vec![1i32]);
    let arr_data = arr.to_data();
    let (real_ffi_array, schema) = dora_operator_api_types::arrow::ffi::to_ffi(&arr_data)
        .expect("to_ffi on valid Int32Array must not fail");
    // Intentional memory leak: we forget the valid FFI_ArrowArray so its
    // release callback is not invoked.  The small tracking allocation is
    // acceptable in test context.
    mem::forget(real_ffi_array);

    // SAFETY: FFI_ArrowArray is repr(C); zeroed gives release = None.
    // The Drop impl for FFI_ArrowArray only calls release when Some, so
    // dropping this is a no-op — no dangling-pointer deref occurs.
    let bad_array: FFI_ArrowArray = unsafe { mem::zeroed() };

    let input = Input {
        id: "test-input".to_owned().into(),
        data_array: Some(bad_array),
        schema,
        metadata: Metadata {
            open_telemetry_context: String::new().into(),
        },
    };
    let mut event = RawEvent {
        input: Some(Box::new(input).into()),
        input_closed: None,
        stop: false,
        error: None,
    };
    let send_output = noop_send_output();

    // SAFETY: operator_context is live (just returned by init), event and
    // send_output outlive this call.
    let OnEventResult { result, status } =
        unsafe { on_event_sym(&mut event, &send_output, operator_context) };

    assert!(
        result.error.is_none(),
        "on_input_parse_error returned an error: {:?}",
        result.error
    );
    assert!(
        matches!(status, DoraStatus::Continue),
        "expected Continue after InputParseError, got {status:?}"
    );

    let drop_result = unsafe { drop_op(operator_context) };
    assert!(
        drop_result.error.is_none(),
        "dora_drop_operator failed: {:?}",
        drop_result.error
    );
}
