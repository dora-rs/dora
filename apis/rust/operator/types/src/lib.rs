#![deny(elided_lifetimes_in_paths)] // required for safer-ffi
#![allow(improper_ctypes_definitions)]
#![allow(clippy::missing_safety_doc)]

pub use arrow;
use adora_arrow_convert::{ArrowData, IntoArrow};
pub use safer_ffi;

use arrow::{
    array::Array,
    ffi::{FFI_ArrowArray, FFI_ArrowSchema},
};
use core::slice;
use safer_ffi::{
    char_p::{self, char_p_boxed},
    closure::ArcDynFn1,
    derive_ReprC, ffi_export,
};
use std::{ops::Deref, path::Path};

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
pub struct AdoraInitOperator {
    pub init_operator: unsafe extern "C" fn() -> AdoraInitResult,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct AdoraInitResult {
    pub result: AdoraResult,
    pub operator_context: *mut std::ffi::c_void,
}
#[derive_ReprC]
#[ffi_export]
#[repr(C)]
pub struct AdoraDropOperator {
    pub drop_operator: unsafe extern "C" fn(operator_context: *mut std::ffi::c_void) -> AdoraResult,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct AdoraResult {
    pub error: Option<safer_ffi::boxed::Box<safer_ffi::String>>,
}

impl AdoraResult {
    pub const SUCCESS: Self = Self { error: None };

    pub fn from_error(error: String) -> Self {
        Self {
            error: Some(Box::new(safer_ffi::String::from(error)).into()),
        }
    }

    pub fn error(&self) -> Option<&str> {
        self.error.as_deref().map(|s| s.deref())
    }

    pub fn into_result(self) -> Result<(), String> {
        match self.error {
            None => Ok(()),
            Some(error) => {
                let converted = safer_ffi::boxed::Box_::into(error);
                Err((*converted).into())
            }
        }
    }
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
pub struct AdoraOnEvent {
    pub on_event: OnEventFn,
}

#[derive_ReprC]
#[ffi_export]
#[repr(transparent)]
pub struct OnEventFn(
    pub  unsafe extern "C" fn(
        event: &mut RawEvent,
        send_output: &SendOutput,
        operator_context: *mut std::ffi::c_void,
    ) -> OnEventResult,
);

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct RawEvent {
    pub input: Option<safer_ffi::boxed::Box<Input>>,
    pub input_closed: Option<safer_ffi::String>,
    pub stop: bool,
    pub error: Option<safer_ffi::String>,
}

#[derive_ReprC]
#[repr(opaque)]
#[derive(Debug)]
pub struct Input {
    pub id: safer_ffi::String,
    pub data_array: Option<FFI_ArrowArray>,
    pub schema: FFI_ArrowSchema,
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
pub struct SendOutput {
    pub send_output: ArcDynFn1<AdoraResult, Output>,
}

#[derive_ReprC]
#[repr(opaque)]
#[derive(Debug)]
pub struct Output {
    pub id: safer_ffi::String,
    pub data_array: FFI_ArrowArray,
    pub schema: FFI_ArrowSchema,
    pub metadata: Metadata,
}

#[derive_ReprC]
#[ffi_export]
#[repr(C)]
#[derive(Debug)]
pub struct OnEventResult {
    pub result: AdoraResult,
    pub status: AdoraStatus,
}

#[derive_ReprC]
#[ffi_export]
#[derive(Debug)]
#[repr(u8)]
pub enum AdoraStatus {
    Continue = 0,
    Stop = 1,
    StopAll = 2,
}

#[ffi_export]
pub fn adora_read_input_id(input: &Input) -> char_p_boxed {
    char_p::new(&*input.id)
}

#[ffi_export]
pub fn adora_free_input_id(_input_id: char_p_boxed) {}

#[ffi_export]
pub fn adora_read_data(input: &mut Input) -> Option<safer_ffi::Vec<u8>> {
    let data_array = input.data_array.take()?;
    let data = unsafe { arrow::ffi::from_ffi(data_array, &input.schema).ok()? };
    let array = ArrowData(arrow::array::make_array(data));
    let bytes: &[u8] = TryFrom::try_from(&array).ok()?;
    Some(bytes.to_owned().into())
}

#[ffi_export]
pub fn adora_free_data(_data: safer_ffi::Vec<u8>) {}

#[ffi_export]
pub unsafe fn adora_send_operator_output(
    send_output: &SendOutput,
    id: safer_ffi::char_p::char_p_ref<'_>,
    data_ptr: *const u8,
    data_len: usize,
) -> AdoraResult {
    let result = || {
        let data = unsafe { slice::from_raw_parts(data_ptr, data_len) };
        let arrow_data = data.to_owned().into_arrow();
        let (data_array, schema) =
            arrow::ffi::to_ffi(&arrow_data.into_data()).map_err(|err| err.to_string())?;
        let output = Output {
            id: id.to_str().to_owned().into(),
            data_array,
            schema,
            metadata: Metadata {
                open_telemetry_context: String::new().into(), // TODO
            },
        };
        Result::<_, String>::Ok(output)
    };
    match result() {
        Ok(output) => send_output.send_output.call(output),
        Err(error) => AdoraResult {
            error: Some(Box::new(safer_ffi::String::from(error)).into()),
        },
    }
}

pub fn generate_headers(target_file: &Path) -> ::std::io::Result<()> {
    ::safer_ffi::headers::builder()
        .to_file(target_file)?
        .generate()
}
