use crate::primitives::*;

/// An array type with a static size
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Array {
    /// The type of the elements
    pub value_type: NestableType,
    /// The number of elements in the array
    pub size: usize,
}

/// A sequence type with an unlimited number of elements
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sequence {
    /// The type of the elements
    pub value_type: NestableType,
}

/// A sequence type with a maximum number of elements
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoundedSequence {
    /// The type of the elements
    pub value_type: NestableType,
    /// The maximum number of elements in the sequence
    pub max_size: usize,
}

/// An array type of a primitive type
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PrimitiveArray {
    /// The type of the elements
    pub value_type: PrimitiveType,
    /// The number of elements in the array
    pub size: usize,
}
