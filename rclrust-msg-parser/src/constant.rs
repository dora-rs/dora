use anyhow::{ensure, Result};
use nom::bytes::complete::is_not;
use nom::character::complete::{char, space0, space1};
use nom::combinator::{eof, recognize};
use nom::multi::separated_list1;
use nom::sequence::tuple;
use rclrust_msg_types::{Constant, ConstantType, PrimitiveType};

use crate::error::RclMsgError;
use crate::{ident, literal, types};

fn validate_value(r#type: ConstantType, value: &str) -> Result<String> {
    match r#type {
        ConstantType::BasicType(t) => {
            let (rest, value) = literal::get_basic_type_literal_parser(t)(value)
                .map_err(|_| RclMsgError::ParseConstantValueError(value.into()))?;
            ensure!(rest.is_empty());
            Ok(value)
        }
        ConstantType::GenericUnboundedString(t) => {
            let (rest, default) = literal::get_string_literal_parser(t.into())(value)
                .map_err(|_| RclMsgError::ParseDefaultValueError(value.into()))?;
            ensure!(rest.is_empty());
            Ok(format!(r##"r#"{}"#"##, default))
        }
        ConstantType::PrimitiveArray(array_t) => match array_t.value_type {
            PrimitiveType::BasicType(t) => {
                let (rest, values) = literal::basic_type_sequence(t, value)
                    .map_err(|_| RclMsgError::ParseDefaultValueError(value.into()))?;
                ensure!(rest.is_empty());
                ensure!(values.len() == array_t.size);

                Ok(format!(
                    "[{}]",
                    values
                        .iter()
                        .map(|v| v.to_string())
                        .collect::<Vec<_>>()
                        .join(", ")
                ))
            }
            PrimitiveType::GenericUnboundedString(_) => {
                let (rest, values) = literal::string_literal_sequence(value)
                    .map_err(|_| RclMsgError::ParseDefaultValueError(value.into()))?;
                ensure!(rest.is_empty());
                Ok(format!(
                    "[{}]",
                    values
                        .iter()
                        .map(|s| format!(r##"r#"{}"#"##, s))
                        .collect::<Vec<_>>()
                        .join(", "),
                ))
            }
        },
    }
}

pub fn constant_def(line: &str) -> Result<Constant> {
    let (_, (r#type, _, name, _, _, _, value, _, _)) = tuple((
        types::parse_constant_type,
        space1,
        ident::constant_name,
        space0,
        char('='),
        space0,
        recognize(separated_list1(space1, is_not(" \t"))),
        space0,
        eof,
    ))(line)
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
    use rclrust_msg_types::*;

    #[test]
    fn parse_member_def_with_default() -> Result<()> {
        let result = constant_def("int32 AAA=30")?;
        assert_eq!(result.name, "AAA");
        assert_eq!(result.r#type, BasicType::I32.into());
        assert_eq!(result.value, "30");
        Ok(())
    }
}
