use std::convert::TryFrom;

use nom::{
    branch::alt,
    bytes::complete::{is_not, tag, tag_no_case, take_while},
    character::complete::{anychar, char, digit1, hex_digit1, none_of, oct_digit1, one_of, space0},
    combinator::{eof, map, map_res, opt, recognize, rest, value, verify},
    multi::{many0, separated_list1},
    number::complete::recognize_float,
    sequence::{delimited, pair, tuple},
    IResult,
};

use crate::types::primitives::{BasicType, GenericString};

pub fn usize_literal(s: &str) -> IResult<&str, usize> {
    map_res(dec_literal, usize::try_from)(s)
}

fn validate_integer_literal<T>(s: &str) -> IResult<&str, String>
where
    T: TryFrom<i128> + ToString,
{
    map_res(integer_literal, |v| T::try_from(v).map(|v| v.to_string()))(s)
}

fn validate_floating_point_literal(s: &str) -> IResult<&str, String> {
    map(recognize_float, |v: &str| v.to_string())(s)
}

fn validate_boolean_literal(s: &str) -> IResult<&str, String> {
    map(bool_literal, |v| v.to_string())(s)
}

pub fn get_basic_type_literal_parser(basic_type: BasicType) -> fn(&str) -> IResult<&str, String> {
    match basic_type {
        BasicType::U8 | BasicType::Char | BasicType::Byte => validate_integer_literal::<u8>,
        BasicType::U16 => validate_integer_literal::<u16>,
        BasicType::U32 => validate_integer_literal::<u32>,
        BasicType::U64 => validate_integer_literal::<u64>,
        BasicType::I8 => validate_integer_literal::<i8>,
        BasicType::I16 => validate_integer_literal::<i16>,
        BasicType::I32 => validate_integer_literal::<i32>,
        BasicType::I64 => validate_integer_literal::<i64>,
        BasicType::F32 | BasicType::F64 => validate_floating_point_literal,
        BasicType::Bool => validate_boolean_literal,
    }
}

pub fn basic_type_sequence(basic_type: BasicType, s: &str) -> IResult<&str, Vec<String>> {
    delimited(
        pair(char('['), space0),
        separated_list1(
            char(','),
            delimited(space0, get_basic_type_literal_parser(basic_type), space0),
        ),
        pair(space0, char(']')),
    )(s)
}

#[inline]
fn flag_if_exist(s: &str) -> IResult<&str, char> {
    map(opt(one_of("+-")), |flag| flag.unwrap_or('+'))(s)
}

fn dec_literal(s: &str) -> IResult<&str, i128> {
    map_res(
        tuple((flag_if_exist, separated_list1(char('_'), digit1))),
        |(flag, digits)| format!("{}{}", flag, digits.join("")).parse::<i128>(),
    )(s)
}

fn integer_literal(s: &str) -> IResult<&str, i128> {
    alt((
        map_res(
            tuple((
                flag_if_exist,
                tag_no_case("0b"),
                separated_list1(char('_'), take_while(|c| c == '0' || c == '1')),
            )),
            |(flag, _, digits)| i128::from_str_radix(&format!("{}{}", flag, digits.join("")), 2),
        ),
        map_res(
            tuple((
                flag_if_exist,
                tag_no_case("0o"),
                separated_list1(char('_'), oct_digit1),
            )),
            |(flag, _, digits)| i128::from_str_radix(&format!("{}{}", flag, digits.join("")), 8),
        ),
        map_res(
            tuple((
                flag_if_exist,
                tag_no_case("0x"),
                separated_list1(char('_'), hex_digit1),
            )),
            |(flag, _, digits)| i128::from_str_radix(&format!("{}{}", flag, digits.join("")), 16),
        ),
        dec_literal,
    ))(s)
}

fn bool_literal(s: &str) -> IResult<&str, bool> {
    alt((
        value(true, alt((tag("true"), tag("1")))),
        value(false, alt((tag("false"), tag("0")))),
    ))(s)
}

#[allow(clippy::type_complexity)]
pub fn get_string_literal_parser(
    string_type: GenericString,
) -> Box<dyn FnMut(&str) -> IResult<&str, String>> {
    match string_type {
        GenericString::String | GenericString::WString => Box::new(string_literal),
        GenericString::BoundedString(max_size) | GenericString::BoundedWString(max_size) => {
            Box::new(move |s| verify(string_literal, |s: &str| s.len() <= max_size)(s))
        }
    }
}

fn string_literal(s: &str) -> IResult<&str, String> {
    alt((
        delimited(
            char('"'),
            map(
                many0(alt((
                    value(r#"""#, tag(r#"\""#)),
                    tag(r#"\"#),
                    recognize(is_not(r#"\""#)),
                ))),
                |v| v.join("").trim().to_string(),
            ),
            char('"'),
        ),
        delimited(
            char('\''),
            map(
                many0(alt((
                    value("'", tag(r#"\'"#)),
                    tag(r#"\"#),
                    recognize(is_not(r#"\'"#)),
                ))),
                |v| v.join("").trim().to_string(),
            ),
            char('\''),
        ),
        value("".to_string(), one_of(r#""'"#)),
        map(
            verify(recognize(many0(anychar)), |v: &str| {
                let v = v.trim();
                !(v.starts_with('"') && v.ends_with('"')
                    || v.starts_with('\'') && v.ends_with('\''))
            }),
            |v: &str| v.trim().to_string(),
        ),
    ))(s)
}

pub fn string_literal_sequence(s: &str) -> IResult<&str, Vec<String>> {
    verify(rest, |v: &str| v.starts_with('[') && v.ends_with(']'))(s)?;

    delimited(
        space0,
        separated_list1(
            char(','),
            delimited(
                space0,
                alt((
                    delimited(
                        char('"'),
                        map(
                            many0(alt((
                                value(r#"""#, tag(r#"\""#)),
                                tag(r#"\"#),
                                recognize(is_not(r#"\""#)),
                            ))),
                            |v| v.join("").trim().to_string(),
                        ),
                        char('"'),
                    ),
                    delimited(
                        char('\''),
                        map(
                            many0(alt((
                                value("'", tag(r#"\'"#)),
                                tag(r#"\"#),
                                recognize(is_not(r#"\'"#)),
                            ))),
                            |v| v.join("").trim().to_string(),
                        ),
                        char('\''),
                    ),
                    map(
                        recognize(pair(none_of("\"',"), opt(is_not(",")))),
                        |s: &str| s.trim().to_string(),
                    ),
                )),
                space0,
            ),
        ),
        tuple((opt(char(',')), space0, eof)),
    )(s.strip_prefix('[').unwrap().strip_suffix(']').unwrap())
}

#[cfg(test)]
mod test {
    use anyhow::Result;

    use super::*;

    #[test]
    fn parse_integer_literal() -> Result<()> {
        assert_eq!(integer_literal("101_010")?.1, 101010);
        Ok(())
    }

    #[test]
    fn parse_bin_literal() -> Result<()> {
        assert_eq!(integer_literal("0b101_010")?.1, 0b101010);
        assert_eq!(integer_literal("+0b101_010")?.1, 0b101010);
        assert_eq!(integer_literal("-0b101_010")?.1, -0b101010);
        Ok(())
    }

    #[test]
    fn parse_oct_literal() -> Result<()> {
        assert_eq!(integer_literal("0o12_345_670")?.1, 0o12345670);
        assert_eq!(integer_literal("+0o12_345_670")?.1, 0o12345670);
        assert_eq!(integer_literal("-0o12_345_670")?.1, -0o12345670);
        Ok(())
    }

    #[test]
    fn parse_dec_literal() -> Result<()> {
        assert_eq!(integer_literal("123_456_789")?.1, 123456789);
        assert_eq!(integer_literal("+123_456_789")?.1, 123456789);
        assert_eq!(integer_literal("-123_456_789")?.1, -123456789);
        Ok(())
    }

    #[test]
    fn parse_hex_literal() -> Result<()> {
        assert_eq!(integer_literal("0x789_aBc")?.1, 0x789abc);
        assert_eq!(integer_literal("+0x789_aBc")?.1, 0x789abc);
        assert_eq!(integer_literal("-0x789_aBc")?.1, -0x789abc);
        Ok(())
    }

    #[test]
    fn parse_bool_literal() -> Result<()> {
        assert!(bool_literal("true")?.1);
        assert!(!bool_literal("false")?.1);
        assert!(bool_literal("1")?.1);
        assert!(!bool_literal("0")?.1);
        Ok(())
    }

    #[test]
    fn parse_integer_sequenc() -> Result<()> {
        assert_eq!(
            basic_type_sequence(BasicType::I8, "[-1, 0x10, 0o10, -0b10]")?.1,
            vec!["-1", "16", "8", "-2"]
        );
        Ok(())
    }

    #[test]
    fn parse_string() -> Result<()> {
        assert_eq!(string_literal(r#""aaa\"aaa" "#)?.1, r#"aaa"aaa"#);
        assert_eq!(string_literal(r#"'aaa\'aaa' "#)?.1, "aaa'aaa");
        Ok(())
    }

    #[test]
    fn parse_string_sequence() -> Result<()> {
        assert_eq!(
            string_literal_sequence(r#"[aaa, "bbb", 'ccc']"#)?.1,
            vec!["aaa", "bbb", "ccc"]
        );
        assert_eq!(
            string_literal_sequence(r#"[aaa, "bbb", 'ccc',]"#)?.1,
            vec!["aaa", "bbb", "ccc"]
        );
        assert_eq!(
            string_literal_sequence(r#"["aaa, \"bbb", 'ccc']"#)?.1,
            vec![r#"aaa, "bbb"#, "ccc"]
        );
        assert_eq!(
            string_literal_sequence(r#"[ aaa , "bbb"  , 'ccc' ]"#)?.1,
            vec!["aaa", "bbb", "ccc"]
        );
        Ok(())
    }
}
