use anyhow::{ensure, Result};
use nom::{
    bytes::complete::is_not,
    character::complete::{space0, space1},
    combinator::{eof, opt, recognize},
    multi::separated_list1,
    sequence::{preceded, tuple},
};

use super::{error::RclMsgError, ident, literal, types};
use crate::types::{primitives::NestableType, Member, MemberType};

fn nestable_type_default(nestable_type: NestableType, default: &str) -> Result<Vec<String>> {
    match nestable_type {
        NestableType::BasicType(t) => {
            let (rest, default) = literal::get_basic_type_literal_parser(t)(default)
                .map_err(|_| RclMsgError::ParseDefaultValueError(default.into()))?;
            ensure!(rest.is_empty());
            Ok(vec![default])
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
            Ok(vec![default])
        }
    }
}

fn array_type_default(value_type: NestableType, default: &str) -> Result<Vec<String>> {
    match value_type {
        NestableType::BasicType(t) => {
            let (rest, default) = literal::basic_type_sequence(t, default)
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
        NestableType::GenericString(_) => {
            let (rest, default) = literal::string_literal_sequence(default)
                .map_err(|_| RclMsgError::ParseDefaultValueError(default.into()))?;
            ensure!(rest.is_empty());
            Ok(default)
        }
    }
}

fn validate_default(r#type: MemberType, default: &str) -> Result<Vec<String>> {
    match r#type {
        MemberType::NestableType(t) => nestable_type_default(t, default),
        MemberType::Array(t) => {
            let default = array_type_default(t.value_type, default)?;
            ensure!(default.len() == t.size);
            Ok(default)
        }
        MemberType::Sequence(t) => array_type_default(t.value_type, default),
        MemberType::BoundedSequence(t) => {
            let default = array_type_default(t.value_type, default)?;
            ensure!(default.len() <= t.max_size);
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
    use anyhow::Result;

    use super::*;
    use crate::types::primitives::BasicType;

    #[test]
    fn parse_member_def() -> Result<()> {
        let result = member_def("int32 aaa")?;
        assert_eq!(result.name, "aaa");
        assert_eq!(result.r#type, BasicType::I32.into());
        Ok(())
    }

    #[test]
    fn parse_member_def_with_default() -> Result<()> {
        let result = member_def("int32 aaa 30")?;
        assert_eq!(result.name, "aaa");
        assert_eq!(result.r#type, BasicType::I32.into());
        assert_eq!(result.default, Some(vec!["30".into()]));
        Ok(())
    }

    #[test]
    fn parse_member_def_with_invalid_default() -> Result<()> {
        assert!(member_def("uint8 aaa -1").is_err());
        assert!(member_def("uint8 aaa 256").is_err());
        Ok(())
    }
}
