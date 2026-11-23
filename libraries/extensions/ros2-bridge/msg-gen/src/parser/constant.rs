use anyhow::{Result, ensure};
use nom::{
    Parser,
    bytes::complete::is_not,
    character::complete::{char, space0, space1},
    combinator::{eof, recognize},
    multi::separated_list1,
};

use super::{error::RclMsgError, ident, literal, types};
use crate::types::{Constant, ConstantType, primitives::PrimitiveType};

fn validate_value(r#type: ConstantType, value: &str) -> Result<Vec<String>> {
    match r#type {
        ConstantType::PrimitiveType(t) => match t {
            PrimitiveType::BasicType(t) => {
                let (rest, value) = literal::get_basic_type_literal_parser(t)(value)
                    .map_err(|_| RclMsgError::ParseConstantValueError(value.into()))?;
                ensure!(rest.is_empty());
                Ok(vec![value])
            }
            PrimitiveType::GenericUnboundedString(t) => {
                let (rest, default) = literal::get_string_literal_parser(t.into())(value)
                    .map_err(|_| RclMsgError::ParseDefaultValueError(value.into()))?;
                ensure!(rest.is_empty());
                Ok(vec![default])
            }
        },
        ConstantType::PrimitiveArray(array_t) => match array_t.value_type {
            PrimitiveType::BasicType(t) => {
                let (rest, values) = literal::basic_type_sequence(t, value)
                    .map_err(|_| RclMsgError::ParseDefaultValueError(value.into()))?;
                ensure!(rest.is_empty());
                ensure!(values.len() == array_t.size);

                Ok(values)
            }
            PrimitiveType::GenericUnboundedString(_) => {
                let (rest, values) = literal::string_literal_sequence(value)
                    .map_err(|_| RclMsgError::ParseDefaultValueError(value.into()))?;
                ensure!(rest.is_empty());
                ensure!(values.len() == array_t.size);

                Ok(values)
            }
        },
    }
}

pub fn constant_def(line: &str) -> Result<Constant> {
    let (_, (r#type, _, name, _, _, _, value, _, _)) = (
        types::parse_constant_type,
        space1,
        ident::constant_name,
        space0,
        char('='),
        space0,
        recognize(separated_list1(space1, is_not(" \t"))),
        space0,
        eof,
    )
        .parse(line)
        .map_err(|e| RclMsgError::ParseConstantError {
            reason: e.to_string(),
            input: line.into(),
        })?;

    Ok(Constant {
        name: name.into(),
        r#type: r#type.clone(),
        value: validate_value(r#type, value)?,
    })
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::types::primitives::BasicType;

    #[test]
    fn parse_member_def_with_default() -> Result<()> {
        let result = constant_def("int32 AAA=30")?;
        assert_eq!(result.name, "AAA");
        assert_eq!(result.r#type, BasicType::I32.into());
        assert_eq!(result.value, vec!["30"]);
        Ok(())
    }

    #[test]
    fn parse_string_array_constant_matching_size() -> Result<()> {
        let result = constant_def(r#"string[3] NAMES=["a", "b", "c"]"#)?;
        assert_eq!(result.name, "NAMES");
        assert_eq!(result.value, vec!["a", "b", "c"]);
        // The declared size matches the literal count, so codegen must not panic.
        let _ = result.r#type.value_tokens(&result.value);
        Ok(())
    }

    #[test]
    fn string_array_constant_with_wrong_element_count_errors() {
        // A fixed-size string-array constant whose literal element count does not
        // match the declared size must be rejected at parse time with a clean
        // error, mirroring the basic-type array branch — not accepted here and
        // then panicked on later in `ConstantType::value_tokens`.
        assert!(constant_def(r#"string[3] NAMES=["a", "b"]"#).is_err());
        assert!(constant_def(r#"string[1] NAMES=["a", "b"]"#).is_err());
    }

    #[test]
    fn basic_type_array_constant_with_wrong_element_count_errors() {
        // Regression guard for the already-correct basic-type branch.
        assert!(constant_def("uint8[3] NUMS=[1, 2]").is_err());
    }
    #[test]
    fn quoted_string_constant_keeps_surrounding_whitespace() {
        // A quoted constant must keep its whitespace exactly (#4164).
        let c = constant_def(r#"string SEP=" ""#).unwrap();
        assert_eq!(c.value, vec![" ".to_string()]);
        let c = constant_def(r#"string[2] XS=[" a", "b "]"#).unwrap();
        assert_eq!(c.value, vec![" a".to_string(), "b ".to_string()]);
    }

    #[test]
    fn out_of_range_float_constant_becomes_infinity() {
        // `1e40` overflows f32; codegen used to panic in proc-macro2. rosidl
        // accepts it as infinity, and the parser stays as lenient (#4162).
        let tokens = |line: &str| {
            let c = constant_def(line).unwrap();
            let tokens = c.r#type.value_tokens(&c.value);
            quote::quote!(#tokens).to_string()
        };
        assert_eq!(tokens("float32 BIG=1e40"), "f32 :: INFINITY");
        assert_eq!(tokens("float32 SMALL=-1e40"), "f32 :: NEG_INFINITY");
        assert_eq!(tokens("float64 BIG=1e400"), "f64 :: INFINITY");
        assert_eq!(
            tokens("float32[2] XS=[1.5, 1e40]"),
            "[1.5f32 , f32 :: INFINITY ,]"
        );
        // In-range values are unchanged.
        assert_eq!(tokens("float64 OK=1.5e3"), "1500f64");
    }

    #[test]
    fn non_finite_float_constants_are_accepted() {
        // rosidl parses floats with Python's `float()`, which accepts
        // `nan` / `inf` / `infinity` in any case and with a sign (#4278).
        let tokens = |line: &str| {
            let c = constant_def(line).unwrap();
            let tokens = c.r#type.value_tokens(&c.value);
            quote::quote!(#tokens).to_string()
        };
        assert_eq!(tokens("float64 UNKNOWN=nan"), "f64 :: NAN");
        assert_eq!(tokens("float32 UNKNOWN=NaN"), "f32 :: NAN");
        assert_eq!(tokens("float64 BIG=inf"), "f64 :: INFINITY");
        assert_eq!(tokens("float32 BIG=+Infinity"), "f32 :: INFINITY");
        assert_eq!(tokens("float64 SMALL=-inf"), "f64 :: NEG_INFINITY");
        assert_eq!(
            tokens("float64[3] XS=[nan, inf, -INF]"),
            "[f64 :: NAN , f64 :: INFINITY , f64 :: NEG_INFINITY ,]"
        );
        // A word that merely starts with `inf` is still not a float.
        assert!(constant_def("float64 X=information").is_err());
        assert!(constant_def("float64 X=nano").is_err());
    }
}
