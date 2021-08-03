use anyhow::{ensure, Result};
use nom::bytes::complete::is_not;
use nom::character::complete::{space0, space1};
use nom::combinator::{eof, opt, recognize};
use nom::multi::separated_list1;
use nom::sequence::{preceded, tuple};
use rclrust_msg_types::{Member, MemberType, NestableType};

use crate::error::RclMsgError;
use crate::{ident, literal, types};

fn nestable_type_default(nestable_type: NestableType, default: &str) -> Result<String> {
    match nestable_type {
        NestableType::BasicType(t) => {
            let (rest, default) = literal::get_basic_type_literal_parser(t)(default)
                .map_err(|_| RclMsgError::ParseDefaultValueError(default.into()))?;
            ensure!(rest.is_empty());
            Ok(default)
        }
        NestableType::NamedType(t) => {
            Err(RclMsgError::InvalidDefaultError(format!("{}", t)).into())
        }
        NestableType::NamespacedType(t) => {
            Err(RclMsgError::InvalidDefaultError(format!("{}", t)).into())
        }
        NestableType::GenericString(t) => {
            let (rest, default) = literal::get_string_literal_parser(t)(default)
                .map_err(|_| RclMsgError::ParseDefaultValueError(default.into()))?;
            ensure!(rest.is_empty());
            Ok(format!(r##"r#"{}"#"##, default))
        }
    }
}

fn array_type_default(value_type: NestableType, default: &str) -> Result<(String, usize)> {
    match value_type {
        NestableType::BasicType(t) => {
            let (rest, default) = literal::basic_type_sequence(t, default)
                .map_err(|_| RclMsgError::ParseDefaultValueError(default.into()))?;
            ensure!(rest.is_empty());
            Ok((
                format!(
                    "[{}]",
                    default
                        .iter()
                        .map(|v| v.to_string())
                        .collect::<Vec<_>>()
                        .join(", ")
                ),
                default.len(),
            ))
        }
        NestableType::NamedType(t) => {
            Err(RclMsgError::InvalidDefaultError(format!("{}", t)).into())
        }
        NestableType::NamespacedType(t) => {
            Err(RclMsgError::InvalidDefaultError(format!("{}", t)).into())
        }
        NestableType::GenericString(_) => {
            let (rest, default) = literal::string_literal_sequence(default)
                .map_err(|_| RclMsgError::ParseDefaultValueError(default.into()))?;
            ensure!(rest.is_empty());
            Ok((
                format!(
                    "[{}]",
                    default
                        .iter()
                        .map(|s| format!(r##"r#"{}"#"##, s))
                        .collect::<Vec<_>>()
                        .join(", ")
                ),
                default.len(),
            ))
        }
    }
}

fn validate_default(r#type: MemberType, default: &str) -> Result<String> {
    match r#type {
        MemberType::BasicType(t) => nestable_type_default(t.into(), default),
        MemberType::NamedType(t) => nestable_type_default(t.into(), default),
        MemberType::NamespacedType(t) => nestable_type_default(t.into(), default),
        MemberType::GenericString(t) => nestable_type_default(t.into(), default),
        MemberType::Array(t) => {
            let (default, size) = array_type_default(t.value_type, default)?;
            ensure!(size == t.size);
            Ok(default)
        }
        MemberType::Sequence(t) => {
            let (default, _) = array_type_default(t.value_type, default)?;
            Ok(default)
        }
        MemberType::BoundedSequence(t) => {
            let (default, size) = array_type_default(t.value_type, default)?;
            ensure!(size <= t.max_size);
            Ok(default)
        }
    }
}

pub fn member_def(line: &str) -> Result<Member> {
    let (_, (r#type, _, name, default, _, _)) = tuple((
        types::parse_member_type,
        space1,
        ident::member_name,
        opt(preceded(
            space1,
            recognize(separated_list1(space1, is_not(" \t"))),
        )),
        space0,
        eof,
    ))(line)
    .map_err(|e| RclMsgError::ParseMemberError {
        input: line.into(),
        reason: e.to_string(),
    })?;

    Ok(Member {
        name: name.into(),
        r#type: r#type.clone(),
        default: match default {
            Some(v) => Some(validate_default(r#type, v)?),
            None => None,
        },
    })
}

#[cfg(test)]
mod test {
    use super::*;
    use anyhow::Result;
    use rclrust_msg_types::BasicType;

    #[test]
    fn parse_member_def() -> Result<()> {
        let result = member_def("int32 aaa")?;
        assert_eq!(result.name, "aaa");
        assert_eq!(result.r#type, BasicType::I32.into());
        assert_eq!(result.default, None);
        Ok(())
    }

    #[test]
    fn parse_member_def_with_default() -> Result<()> {
        let result = member_def("int32 aaa 30")?;
        assert_eq!(result.name, "aaa");
        assert_eq!(result.r#type, BasicType::I32.into());
        assert_eq!(result.default, Some("30".into()));
        Ok(())
    }

    #[test]
    fn parse_member_def_with_invalid_default() -> Result<()> {
        assert!(member_def("uint8 aaa -1").is_err());
        assert!(member_def("uint8 aaa 256").is_err());
        Ok(())
    }
}
