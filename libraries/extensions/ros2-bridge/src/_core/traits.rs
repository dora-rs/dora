use std::convert::TryInto;

use super::string::U16String;
use array_init::array_init;

pub trait MessageT: Default + Send + Sync {
    type Raw: FFIToRust<Target = Self> + Send + Sync;
    type RawRef: FFIFromRust<From = Self>;

    unsafe fn from_raw(from: &Self::Raw) -> Self {
        from.to_rust()
    }

    unsafe fn to_raw_ref(&self) -> Self::RawRef {
        Self::RawRef::from_rust(self)
    }
}

pub trait ActionT: Send {
    type Goal: MessageT;
    type Result: MessageT;
    type Feedback: MessageT;
    type SendGoal;
    type GetResult;
    type FeedbackMessage: MessageT;
}

// I was going to use `std::default::Default`, however generic arrays do not implement `std::default::Default`.
pub trait InternalDefault {
    fn _default() -> Self;
}

impl<T> InternalDefault for Vec<T> {
    fn _default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> InternalDefault for [T; N]
where
    T: InternalDefault + std::fmt::Debug,
{
    fn _default() -> Self {
        array_init(|_| InternalDefault::_default())
    }
}

macro_rules! impl_trait {
    ($type: ty) => {
        impl InternalDefault for $type {
            fn _default() -> Self {
                Self::default()
            }
        }
    };
}

impl_trait!(i8);
impl_trait!(i16);
impl_trait!(i32);
impl_trait!(i64);
impl_trait!(u8);
impl_trait!(u16);
impl_trait!(u32);
impl_trait!(u64);
impl_trait!(f32);
impl_trait!(f64);
impl_trait!(bool);
impl_trait!(String);
impl_trait!(U16String);

pub trait FFIToRust {
    type Target;

    unsafe fn to_rust(&self) -> Self::Target;
}

impl<T, const N: usize> FFIToRust for [T; N]
where
    T: FFIToRust,
    T::Target: std::fmt::Debug,
{
    type Target = [T::Target; N];

    unsafe fn to_rust(&self) -> <Self as FFIToRust>::Target {
        self.iter()
            .map(|v| v.to_rust())
            .collect::<Vec<_>>()
            .try_into()
            .unwrap()
    }
}

pub trait FFIFromRust {
    type From;

    unsafe fn from_rust(from: &Self::From) -> Self;
}

impl<T, const N: usize> FFIFromRust for [T; N]
where
    T: FFIFromRust + std::fmt::Debug,
{
    type From = [T::From; N];

    unsafe fn from_rust(from: &Self::From) -> Self {
        from.iter()
            .map(|v| FFIFromRust::from_rust(v))
            .collect::<Vec<_>>()
            .try_into()
            .unwrap()
    }
}
