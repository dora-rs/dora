use std::os::raw::c_void;
use widestring::U16String;

pub trait MessageT: Default {
    type Raw: RawMessageT;
    type RawRef: RawMessageRefT;

    fn type_support() -> *const c_void;

    unsafe fn from_raw(from: &Self::Raw) -> Self;

    unsafe fn to_raw_ref(&self) -> Self::RawRef;
}

pub trait RawMessageT: FFIToRust + Default {}

pub trait RawMessageRefT: FFIFromRust {}

pub trait ServiceT {
    type Request: MessageT;
    type Response: MessageT;

    fn type_support() -> *const c_void;
}

pub trait ActionT {
    type Goal: MessageT;
    type Result: MessageT;
    type Feedback: MessageT;
    type SendGoal: ServiceT;
    type GetResult: ServiceT;
    type FeedbackMessage: MessageT;

    fn type_support() -> *const c_void;
}

pub trait ZeroInit {
    fn zero_init() -> Self;
}

impl ZeroInit for String {
    fn zero_init() -> Self {
        "".into()
    }
}

impl ZeroInit for U16String {
    fn zero_init() -> Self {
        Self::new()
    }
}

impl<T> ZeroInit for Vec<T> {
    fn zero_init() -> Self {
        Self::new()
    }
}

pub trait FFIToRust {
    type Target;

    unsafe fn to_rust(&self) -> Self::Target;
}

pub trait FFIFromRust {
    type From;

    fn from_rust(from: &Self::From) -> Self;
}

macro_rules! impl_traits_to_primitive {
    ($type: ty) => {
        impl ZeroInit for $type {
            fn zero_init() -> Self {
                Self::default()
            }
        }

        impl FFIToRust for $type {
            type Target = Self;

            unsafe fn to_rust(&self) -> Self::Target {
                *self
            }
        }
    };
}

impl_traits_to_primitive!(i8);
impl_traits_to_primitive!(i16);
impl_traits_to_primitive!(i32);
impl_traits_to_primitive!(i64);
impl_traits_to_primitive!(u8);
impl_traits_to_primitive!(u16);
impl_traits_to_primitive!(u32);
impl_traits_to_primitive!(u64);
impl_traits_to_primitive!(f32);
impl_traits_to_primitive!(f64);
impl_traits_to_primitive!(bool);
