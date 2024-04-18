use anyhow::anyhow;
use nom::{
    branch::alt,
    bytes::complete::tag,
    character::complete::{char, space1},
    combinator::{eof, map, map_res, opt, peek},
    sequence::{delimited, pair, preceded, tuple},
    IResult,
};

use super::{
    ident::{message_name, package_name},
    literal::usize_literal,
};
use crate::types::{
    primitives::*,
    sequences::{Array, BoundedSequence, PrimitiveArray, Sequence},
    ConstantType, MemberType,
};

pub fn parse_member_type(s: &str) -> IResult<&str, MemberType> {
    map_res(
        tuple((
            nestable_type,
            opt(delimited(
                char('['),
                pair(opt(tag("<=")), opt(usize_literal)),
                char(']'),
            )),
            peek(alt((space1, eof))),
        )),
        |(value_type, seq_info, _)| {
            Ok(match seq_info {
                None => value_type.into(),
                Some((None, None)) => Sequence { value_type }.into(),
                Some((None, Some(size))) => Array { value_type, size }.into(),
                Some((Some(_), Some(size))) => BoundedSequence {
                    value_type,
                    max_size: size,
                }
                .into(),
                Some((Some(_), None)) => {
                    return Err(anyhow!("max_size should be specified"));
                }
            })
        },
    )(s)
}

pub fn parse_constant_type(s: &str) -> IResult<&str, ConstantType> {
    map(
        tuple((
            primitive_type,
            opt(delimited(char('['), usize_literal, char(']'))),
            peek(alt((space1, eof))),
        )),
        |(value_type, size, _)| {
            size.map_or_else(
                || value_type.into(),
                |size| PrimitiveArray { value_type, size }.into(),
            )
        },
    )(s)
}

fn basic_type(s: &str) -> IResult<&str, BasicType> {
    map(
        alt((
            tag("uint8"),
            tag("uint16"),
            tag("uint32"),
            tag("uint64"),
            tag("int8"),
            tag("int16"),
            tag("int32"),
            tag("int64"),
            tag("int64"),
            tag("int64"),
            tag("float32"),
            tag("float64"),
            tag("bool"),
            tag("char"),
            tag("byte"),
        )),
        |s| BasicType::parse(s).unwrap(),
    )(s)
}

fn named_type(s: &str) -> IResult<&str, NamedType> {
    map(message_name, |name| NamedType(name.into()))(s)
}

fn namespaced_type(s: &str) -> IResult<&str, NamespacedType> {
    map(
        tuple((package_name, char('/'), message_name)),
        |(package, _, name)| NamespacedType {
            package: package.into(),
            namespace: "msg".into(),
            name: name.into(),
        },
    )(s)
}

fn generic_string(s: &str) -> IResult<&str, GenericString> {
    map(
        pair(
            alt((tag("string"), tag("wstring"))),
            opt(preceded(tag("<="), usize_literal)),
        ),
        |(type_str, array_info)| {
            array_info.map_or_else(
                || match type_str {
                    "string" => GenericString::String,
                    "wstring" => GenericString::WString,
                    _ => unreachable!(),
                },
                |max_size| match type_str {
                    "string" => GenericString::BoundedString(max_size),
                    "wstring" => GenericString::BoundedWString(max_size),
                    _ => unreachable!(),
                },
            )
        },
    )(s)
}

fn generic_unbounded_string(s: &str) -> IResult<&str, GenericUnboundedString> {
    map(
        alt((tag("string"), tag("wstring"))),
        |type_str| match type_str {
            "string" => GenericUnboundedString::String,
            "wstring" => GenericUnboundedString::WString,
            _ => unreachable!(),
        },
    )(s)
}

fn nestable_type(s: &str) -> IResult<&str, NestableType> {
    alt((
        map(basic_type, |type_| type_.into()),
        map(generic_string, |type_| type_.into()),
        map(namespaced_type, |type_| type_.into()),
        map(named_type, |type_| type_.into()),
    ))(s)
}

fn primitive_type(s: &str) -> IResult<&str, PrimitiveType> {
    alt((
        map(basic_type, |type_| type_.into()),
        map(generic_unbounded_string, |type_| type_.into()),
    ))(s)
}

#[cfg(test)]
mod test {
    use anyhow::Result;

    use super::*;

    #[test]
    fn test_parse_member_type_basic_type() -> Result<()> {
        assert_eq!(parse_member_type("int8")?.1, BasicType::I8.into());
        assert_eq!(parse_member_type("int16")?.1, BasicType::I16.into());
        assert_eq!(parse_member_type("int32")?.1, BasicType::I32.into());
        assert_eq!(parse_member_type("int64")?.1, BasicType::I64.into());
        assert_eq!(parse_member_type("uint8")?.1, BasicType::U8.into());
        assert_eq!(parse_member_type("uint16")?.1, BasicType::U16.into());
        assert_eq!(parse_member_type("uint32")?.1, BasicType::U32.into());
        assert_eq!(parse_member_type("uint64")?.1, BasicType::U64.into());
        assert_eq!(parse_member_type("float32")?.1, BasicType::F32.into());
        assert_eq!(parse_member_type("float64")?.1, BasicType::F64.into());
        assert_eq!(parse_member_type("bool")?.1, BasicType::Bool.into());
        assert_eq!(parse_member_type("char")?.1, BasicType::Char.into());
        assert_eq!(parse_member_type("byte")?.1, BasicType::Byte.into());
        Ok(())
    }

    #[test]
    fn test_parse_member_type_named_type() -> Result<()> {
        assert_eq!(parse_member_type("ABC")?.1, NamedType("ABC".into()).into());
        Ok(())
    }

    #[test]
    fn test_parse_member_type_namespaced_type() -> Result<()> {
        assert_eq!(
            parse_member_type("std_msgs/Bool")?.1,
            NamespacedType {
                package: "std_msgs".into(),
                namespace: "msg".into(),
                name: "Bool".into()
            }
            .into()
        );
        Ok(())
    }

    #[test]
    fn test_parse_member_type_generic_string() -> Result<()> {
        assert_eq!(parse_member_type("string")?.1, GenericString::String.into());
        assert_eq!(
            parse_member_type("wstring")?.1,
            GenericString::WString.into()
        );
        assert_eq!(
            parse_member_type("string<=5")?.1,
            GenericString::BoundedString(5).into()
        );
        assert_eq!(
            parse_member_type("wstring<=5")?.1,
            GenericString::BoundedWString(5).into()
        );
        Ok(())
    }

    #[test]
    fn test_parse_member_type_array() -> Result<()> {
        assert_eq!(
            parse_member_type("string[5]")?.1,
            Array {
                value_type: GenericString::String.into(),
                size: 5,
            }
            .into()
        );
        assert_eq!(
            parse_member_type("string<=6[5]")?.1,
            Array {
                value_type: GenericString::BoundedString(6).into(),
                size: 5,
            }
            .into()
        );
        Ok(())
    }

    #[test]
    fn test_parse_member_type_sequence() -> Result<()> {
        assert_eq!(
            parse_member_type("string[]")?.1,
            Sequence {
                value_type: GenericString::String.into(),
            }
            .into()
        );
        assert_eq!(
            parse_member_type("string<=6[]")?.1,
            Sequence {
                value_type: GenericString::BoundedString(6).into(),
            }
            .into()
        );
        Ok(())
    }

    #[test]
    fn test_parse_member_type_bounded_sequence() -> Result<()> {
        assert_eq!(
            parse_member_type("string[<=5]")?.1,
            BoundedSequence {
                value_type: GenericString::String.into(),
                max_size: 5,
            }
            .into()
        );
        assert_eq!(
            parse_member_type("string<=6[<=5]")?.1,
            BoundedSequence {
                value_type: GenericString::BoundedString(6).into(),
                max_size: 5,
            }
            .into()
        );
        Ok(())
    }

    #[test]
    fn test_parse_constant_type_basic_type() -> Result<()> {
        assert_eq!(parse_constant_type("int8")?.1, BasicType::I8.into());
        assert_eq!(parse_constant_type("int16")?.1, BasicType::I16.into());
        assert_eq!(parse_constant_type("int32")?.1, BasicType::I32.into());
        assert_eq!(parse_constant_type("int64")?.1, BasicType::I64.into());
        assert_eq!(parse_constant_type("uint8")?.1, BasicType::U8.into());
        assert_eq!(parse_constant_type("uint16")?.1, BasicType::U16.into());
        assert_eq!(parse_constant_type("uint32")?.1, BasicType::U32.into());
        assert_eq!(parse_constant_type("uint64")?.1, BasicType::U64.into());
        assert_eq!(parse_constant_type("float32")?.1, BasicType::F32.into());
        assert_eq!(parse_constant_type("float64")?.1, BasicType::F64.into());
        assert_eq!(parse_constant_type("bool")?.1, BasicType::Bool.into());
        assert_eq!(parse_constant_type("char")?.1, BasicType::Char.into());
        assert_eq!(parse_constant_type("byte")?.1, BasicType::Byte.into());
        Ok(())
    }

    #[test]
    fn test_parse_constant_type_named_type() -> Result<()> {
        assert!(parse_constant_type("ABC").is_err());
        Ok(())
    }

    #[test]
    fn test_parse_constant_type_namespaced_type() -> Result<()> {
        assert!(parse_constant_type("std_msgs/Bool").is_err());
        Ok(())
    }

    #[test]
    fn test_parse_constant_type_generic_string() -> Result<()> {
        assert_eq!(
            parse_constant_type("string")?.1,
            GenericUnboundedString::String.into()
        );
        assert_eq!(
            parse_constant_type("wstring")?.1,
            GenericUnboundedString::WString.into()
        );
        assert!(parse_constant_type("string<=5").is_err());
        assert!(parse_constant_type("wstring<=5").is_err());
        Ok(())
    }

    #[test]
    fn test_parse_constant_type_array() -> Result<()> {
        assert_eq!(
            parse_constant_type("string[5]")?.1,
            PrimitiveArray {
                value_type: GenericUnboundedString::String.into(),
                size: 5,
            }
            .into()
        );
        assert!(parse_constant_type("string<=6[5]").is_err());
        Ok(())
    }

    #[test]
    fn test_parse_constant_type_sequence() -> Result<()> {
        assert!(parse_constant_type("string[]").is_err());
        assert!(parse_constant_type("string<=6[]").is_err());
        Ok(())
    }

    #[test]
    fn test_parse_const_type_bounded_sequence() -> Result<()> {
        assert!(parse_constant_type("string[<=5]").is_err());
        assert!(parse_constant_type("string<=6[<=5]").is_err());
        Ok(())
    }
}
