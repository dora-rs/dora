pub use safer_ffi;
use safer_ffi::{closure::RefDynFnMut1, derive_ReprC};

#[derive_ReprC]
#[repr(transparent)]
pub struct DoraInitOperator(pub unsafe extern "C" fn() -> InitResult);

#[derive_ReprC]
#[repr(C)]
#[derive(Debug)]
pub struct InitResult {
    pub result: DoraResult,
    pub operator_context: *mut std::ffi::c_void,
}
#[derive_ReprC]
#[repr(transparent)]
pub struct DoraDropOperator(
    pub unsafe extern "C" fn(operator_context: *mut std::ffi::c_void) -> DoraResult,
);

#[derive_ReprC]
#[repr(C)]
#[derive(Debug)]
pub struct DoraResult {
    pub error: Option<safer_ffi::String>,
}

#[derive_ReprC]
#[repr(transparent)]
pub struct DoraOnInput(
    pub  unsafe extern "C" fn(
        input: &Input,
        send_output: SendOutput<'_>,
        operator_context: *mut std::ffi::c_void,
    ) -> OnInputResult,
);

#[derive_ReprC]
#[repr(C)]
#[derive(Debug)]
pub struct Input {
    pub id: safer_ffi::String,
    pub data: safer_ffi::Vec<u8>,
    pub metadata: Metadata,
}

#[derive_ReprC]
#[repr(C)]
#[derive(Debug)]
pub struct Metadata {
    pub open_telemetry_context: safer_ffi::String,
}

#[derive_ReprC]
#[repr(transparent)]
pub struct SendOutput<'a>(pub RefDynFnMut1<'a, DoraResult, Output>);

#[derive_ReprC]
#[repr(C)]
#[derive(Debug)]
pub struct Output {
    pub id: safer_ffi::String,
    pub data: safer_ffi::Vec<u8>,
    pub metadata: Metadata,
}

#[derive_ReprC]
#[repr(C)]
#[derive(Debug)]
pub struct OnInputResult {
    pub result: DoraResult,
    pub status: DoraStatus,
}

#[derive_ReprC]
#[derive(Debug)]
#[repr(u8)]
pub enum DoraStatus {
    Continue = 0,
    Stop = 1,
}
