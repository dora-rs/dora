use anyhow::{ensure, Result};
use nom::{
    bytes::complete::is_not,
    character::complete::{char, space0, space1},
    combinator::{eof, recognize},
    multi::separated_list1,
    sequence::tuple,
};

use super::{error::RclMsgError, ident, literal, types};
use crate::types::{primitives::PrimitiveType, Constant, ConstantType};

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
                Ok(values)
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
    use crate::types::primitives::BasicType;

    #[test]
    fn parse_member_def_with_default() -> Result<()> {
        let result = constant_def("int32 AAA=30")?;
        assert_eq!(result.name, "AAA");
        assert_eq!(result.r#type, BasicType::I32.into());
        assert_eq!(result.value, vec!["30"]);
        Ok(())
    }
}
