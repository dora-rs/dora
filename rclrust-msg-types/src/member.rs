use crate::define_enum_from;
use crate::primitives::*;
use crate::sequences::*;

/// A type which is available for member
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MemberType {
    BasicType(BasicType),
    NamedType(NamedType),
    NamespacedType(NamespacedType),
    GenericString(GenericString),
    Array(Array),
    Sequence(Sequence),
    BoundedSequence(BoundedSequence),
}

define_enum_from!(MemberType, BasicType, Self::BasicType);
define_enum_from!(MemberType, NamedType, Self::NamedType);
define_enum_from!(MemberType, NamespacedType, Self::NamespacedType);
define_enum_from!(MemberType, GenericString, Self::GenericString);
define_enum_from!(MemberType, Array, Self::Array);
define_enum_from!(MemberType, Sequence, Self::Sequence);
define_enum_from!(MemberType, BoundedSequence, Self::BoundedSequence);

impl MemberType {
    pub fn inner_type(self) -> NestableType {
        match self {
            Self::BasicType(t) => t.into(),
            Self::NamedType(t) => t.into(),
            Self::NamespacedType(t) => t.into(),
            Self::GenericString(t) => t.into(),
            Self::Array(t) => t.value_type,
            Self::Sequence(t) => t.value_type,
            Self::BoundedSequence(t) => t.value_type,
        }
    }
}

impl From<NestableType> for MemberType {
    fn from(t: NestableType) -> Self {
        match t {
            NestableType::BasicType(t) => Self::BasicType(t),
            NestableType::NamedType(t) => Self::NamedType(t),
            NestableType::NamespacedType(t) => Self::NamespacedType(t),
            NestableType::GenericString(t) => Self::GenericString(t),
        }
    }
}
