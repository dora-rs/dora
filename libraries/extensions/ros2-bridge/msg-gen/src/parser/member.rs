use anyhow::{Result, ensure};
use nom::{
    Parser,
    bytes::complete::is_not,
    character::complete::{space0, space1},
    combinator::{eof, opt, recognize},
    multi::separated_list1,
    sequence::preceded,
};

use super::{error::RclMsgError, ident, literal, types};
use crate::types::{
    Member, MemberType,
    primitives::{GenericString, NestableType},
};

fn nestable_type_default(nestable_type: NestableType, default: &str) -> Result<Vec<String>> {
    match nestable_type {
        NestableType::BasicType(t) => {
            let (rest, default) = literal::get_basic_type_literal_parser(t)(default)
                .map_err(|_| RclMsgError::ParseDefaultValueError(default.into()))?;
            ensure!(rest.is_empty());
            Ok(vec![default])
        }
        NestableType::NamedType(t) => Err(RclMsgError::InvalidDefaultError(format!("{t}")).into()),
        NestableType::NamespacedType(t) => {
            Err(RclMsgError::InvalidDefaultError(format!("{t}")).into())
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
        NestableType::NamedType(t) => Err(RclMsgError::InvalidDefaultError(format!("{t}")).into()),
        NestableType::NamespacedType(t) => {
            Err(RclMsgError::InvalidDefaultError(format!("{t}")).into())
        }
        NestableType::GenericString(t) => {
            let (rest, default) = literal::string_literal_sequence(default)
                .map_err(|_| RclMsgError::ParseDefaultValueError(default.into()))?;
            ensure!(rest.is_empty());
            // Enforce the per-element length bound for bounded strings, matching
            // the scalar path in `nestable_type_default` (which threads the bound
            // through `get_string_literal_parser`). Without this, an over-long
            // element in a `string<=N[..]` / `wstring<=N[..]` default is silently
            // accepted.
            match t {
                GenericString::BoundedString(max) => {
                    ensure!(default.iter().all(|s| s.len() <= max));
                }
                GenericString::BoundedWString(max) => {
                    ensure!(default.iter().all(|s| s.encode_utf16().count() <= max));
                }
                GenericString::String | GenericString::WString => {}
            }
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
    let (_, (r#type, _, name, default, _, _)) = (
        types::parse_member_type,
        space1,
        ident::member_name,
        opt(preceded(
            space1,
            recognize(separated_list1(space1, is_not(" \t"))),
        )),
        space0,
        eof,
    )
        .parse(line)
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

    #[test]
    fn bounded_string_array_enforces_per_element_bound() -> Result<()> {
        // Scalar bound is enforced ("hello" is 5 bytes > 3).
        assert!(member_def(r#"string<=3 aaa "hello""#).is_err());
        // The same bound must apply to each element of an array default.
        assert!(member_def(r#"string<=3[2] aaa ["hello", "hi"]"#).is_err());
        // ...and of a sequence default.
        assert!(member_def(r#"string<=3[] aaa ["hello", "hi"]"#).is_err());
        // Within-bound elements are still accepted.
        let result = member_def(r#"string<=3[2] aaa ["ok", "hi"]"#)?;
        assert_eq!(result.default, Some(vec!["ok".into(), "hi".into()]));
        Ok(())
    }

    #[test]
    fn bounded_wstring_array_enforces_per_element_bound() -> Result<()> {
        // wstring bound is measured in UTF-16 code units.
        assert!(member_def(r#"wstring<=3[2] aaa ["hello", "hi"]"#).is_err());
        let result = member_def(r#"wstring<=3[2] aaa ["ok", "hi"]"#)?;
        assert_eq!(result.default, Some(vec!["ok".into(), "hi".into()]));
        Ok(())
    }
    #[test]
    fn empty_sequence_default_is_accepted() -> Result<()> {
        // `[]` is a valid default for `T[]` and `T[<=N]` (#4163).
        for line in [
            "int32[] x []",
            "string[] y []",
            "int32[<=3] z []",
            "string<=4[<=2] w [ ]",
        ] {
            let result = member_def(line)?;
            assert_eq!(result.default, Some(Vec::new()), "{line}");
            // Codegen turns it into an empty `vec![]` without panicking.
            let tokens = result.r#type.value_tokens(&[]);
            assert_eq!(quote::quote!(#tokens).to_string(), "vec ! []", "{line}");
        }
        Ok(())
    }

    #[test]
    fn empty_default_is_rejected_for_fixed_size_arrays() {
        assert!(member_def("int32[3] x []").is_err());
        assert!(member_def("string[3] y []").is_err());
    }

    #[test]
    fn out_of_range_float_default_becomes_infinity() {
        // #4162: no panic in codegen, and the `.msg` still parses (the Python
        // bridge parses every package at runtime and must not fail on it).
        let m = member_def("float64 x -1e400").unwrap();
        let tokens = m.r#type.value_tokens(m.default.as_ref().unwrap());
        assert_eq!(quote::quote!(#tokens).to_string(), "f64 :: NEG_INFINITY");
        let m = member_def("float32[] xs [0.0, 1e39]").unwrap();
        let tokens = m.r#type.value_tokens(m.default.as_ref().unwrap());
        assert_eq!(
            quote::quote!(#tokens).to_string(),
            "vec ! [0f32 , f32 :: INFINITY ,]"
        );
    }
    #[test]
    fn quoted_string_default_keeps_surrounding_whitespace() -> Result<()> {
        // Quoting is the only way to keep leading/trailing whitespace in a
        // `.msg` value, so it must round-trip exactly (#4164).
        let result = member_def(r#"string x "  padded  ""#)?;
        assert_eq!(result.default, Some(vec!["  padded  ".into()]));
        let result = member_def("string x ' a '")?;
        assert_eq!(result.default, Some(vec![" a ".into()]));
        let result = member_def(r#"string[] xs [" a", 'b ', c ]"#)?;
        assert_eq!(
            result.default,
            Some(vec![" a".into(), "b ".into(), "c".into()])
        );
        Ok(())
    }

    #[test]
    fn bounded_string_default_counts_quoted_whitespace() -> Result<()> {
        // `"  ab  "` is 6 bytes, so it must not fit a 4-byte bound (#4164).
        assert!(member_def(r#"string<=4 x "  ab  ""#).is_err());
        assert!(member_def(r#"string<=4[1] x ["  ab  "]"#).is_err());
        let result = member_def(r#"string<=4 x " ab ""#)?;
        assert_eq!(result.default, Some(vec![" ab ".into()]));
        Ok(())
    }
}
