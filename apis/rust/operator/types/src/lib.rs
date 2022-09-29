#![deny(elided_lifetimes_in_paths)] // required for safer-ffi
#![allow(non_snake_case)] // required for safer-ffi

pub use safer_ffi;
use safer_ffi::{
    closure::{ArcDynFn1, BoxDynFnMut0},
    derive_ReprC, ffi_export,
};
use std::path::Path;

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
pub struct DoraInitOperator {
    pub init_operator: unsafe extern "C" fn() -> DoraInitResult,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct DoraInitResult {
    pub result: DoraResult,
    pub operator_context: *mut std::ffi::c_void,
}
#[derive_ReprC]
#[ffi_export]
#[repr(C)]
pub struct DoraDropOperator {
    pub drop_operator: unsafe extern "C" fn(operator_context: *mut std::ffi::c_void) -> DoraResult,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct DoraResult {
    pub error: Option<safer_ffi::String>,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
pub struct DoraOnInput {
    pub on_input: OnInputFn,
}

#[derive_ReprC]
#[ffi_export]
#[repr(transparent)]
pub struct OnInputFn(
    pub  unsafe extern "C" fn(
        input: &Input,
        send_output: &PrepareOutput,
        operator_context: *mut std::ffi::c_void,
    ) -> OnInputResult,
);

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct Input {
    pub id: safer_ffi::String,
    pub data: safer_ffi::Vec<u8>,
    pub metadata: Metadata,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct Metadata {
    pub open_telemetry_context: safer_ffi::String,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
pub struct PrepareOutput {
    pub prepare_output: ArcDynFn1<PrepareOutputResult, OutputMetadata>,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct OutputMetadata {
    pub id: safer_ffi::String,
    pub metadata: Metadata,
    pub data_len: usize,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct PrepareOutputResult {
    pub result: DoraResult,
    pub output: Output,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct Output {
    pub data_mut: BoxDynFnMut0<safer_ffi::slice::slice_raw<u8>>,
    pub send: BoxDynFnMut0<DoraResult>,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct OnInputResult {
    pub result: DoraResult,
    pub status: DoraStatus,
}

#[derive_ReprC]
#[ffi_export]
#[derive(Debug)]
#[repr(u8)]
pub enum DoraStatus {
    Continue = 0,
    Stop = 1,
}

pub fn generate_headers(target_file: &Path) -> ::std::io::Result<()> {
    ::safer_ffi::headers::builder()
        .to_file(target_file)?
        .generate()
}
