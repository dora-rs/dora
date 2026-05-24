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
//! * Passing a structurally-valid `FFI_ArrowArray` for Int32 (n_buffers = 2,
//!   non-null buffers pointer, length = 1) with a null values slot causes
//!   `arrow::ffi::from_ffi` to return `Err("null buffer at position 1")`,
//!   which `raw::dora_on_event` converts to `Event::InputParseError` and
//!   dispatches to `OperatorWrapper::on_event` → `ffi::on_input_parse_error`
//!   (the cxx-bridge call site added in #1879).
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

    // Schema and array are independent objects in the Arrow C Data Interface;
    // we only need the type metadata (schema) here.
    let arr = Int32Array::from(vec![1i32]);
    let arr_data = arr.to_data();
    let (_ffi_array, schema) = dora_operator_api_types::arrow::ffi::to_ffi(&arr_data)
        .expect("to_ffi on valid Int32Array must not fail");
    drop(_ffi_array);

    // Construct a structurally-valid FFI_ArrowArray for Int32 that causes
    // arrow::ffi::from_ffi to return Err rather than panic.
    //
    // Per Arrow C Data Interface, Int32 requires n_buffers == 2:
    //   buffers[0] = validity bitmap — null is valid (means no nulls)
    //   buffers[1] = values buffer  — null + non-zero byte length → Err
    //
    // The `buffers` field must be a non-null pointer; FFI_ArrowArray::buffer()
    // asserts !buffers.is_null() before from_ffi can inspect the per-slot
    // values.  length = 1 keeps the computed byte-size > 0 so from_ffi
    // returns Err instead of Ok(empty).  Drop is a no-op when release == None.
    let buffer_ptrs: [*const std::ffi::c_void; 2] = [std::ptr::null(), std::ptr::null()];
    // SAFETY: FFI_ArrowArray is repr(C). We set n_buffers, buffers, and length
    // to structurally-valid values; all other fields are zeroed (null/false/0).
    let mut bad_array: FFI_ArrowArray = unsafe { mem::zeroed() };
    bad_array.n_buffers = 2;
    bad_array.buffers = buffer_ptrs.as_ptr() as *mut *const std::ffi::c_void;
    bad_array.length = 1;

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
