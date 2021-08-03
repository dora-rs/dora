use crate::define_enum_from;
use crate::primitives::{BasicType, GenericUnboundedString, PrimitiveType};
use crate::sequences::PrimitiveArray;

/// A type which is available for constant
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConstantType {
    BasicType(BasicType),
    GenericUnboundedString(GenericUnboundedString),
    PrimitiveArray(PrimitiveArray),
}

define_enum_from!(ConstantType, BasicType, Self::BasicType);
define_enum_from!(
    ConstantType,
    GenericUnboundedString,
    Self::GenericUnboundedString
);
define_enum_from!(ConstantType, PrimitiveArray, Self::PrimitiveArray);

impl From<PrimitiveType> for ConstantType {
    fn from(t: PrimitiveType) -> Self {
        match t {
            PrimitiveType::BasicType(t) => Self::BasicType(t),
            PrimitiveType::GenericUnboundedString(t) => Self::GenericUnboundedString(t),
        }
    }
}
