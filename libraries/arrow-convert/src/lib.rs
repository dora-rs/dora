use std::ops::{Deref, DerefMut};

use arrow::array::Array;

mod from_impls;
mod into_impls;

pub trait IntoArrow {
    type A: Array;

    fn into_arrow(self) -> Self::A;
}

#[derive(Debug)]
pub struct ArrowData(pub arrow::array::ArrayRef);

impl Deref for ArrowData {
    type Target = arrow::array::ArrayRef;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for ArrowData {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
