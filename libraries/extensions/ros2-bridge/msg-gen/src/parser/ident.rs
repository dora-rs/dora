use nom::{
    branch::alt,
    character::complete::{alphanumeric0, char, one_of},
    combinator::{opt, recognize},
    multi::{many1, separated_list0, separated_list1},
    sequence::{pair, tuple},
    IResult,
};

fn upperalpha(s: &str) -> IResult<&str, char> {
    one_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ")(s)
}

fn loweralpha(s: &str) -> IResult<&str, char> {
    one_of("abcdefghijklmnopqrstuvwxyz")(s)
}

fn numeric(s: &str) -> IResult<&str, char> {
    one_of("0123456789")(s)
}

pub fn package_name(s: &str) -> IResult<&str, &str> {
    recognize(tuple((
        loweralpha,
        opt(char('_')),
        separated_list1(char('_'), many1(alt((loweralpha, numeric)))),
    )))(s)
}

pub fn member_name(s: &str) -> IResult<&str, &str> {
    recognize(tuple((
        loweralpha,
        opt(char('_')),
        separated_list0(char('_'), many1(alt((loweralpha, numeric)))),
    )))(s)
}

pub fn message_name(s: &str) -> IResult<&str, &str> {
    recognize(pair(upperalpha, alphanumeric0))(s)
}

pub fn constant_name(s: &str) -> IResult<&str, &str> {
    recognize(separated_list1(
        char('_'),
        many1(alt((upperalpha, numeric))),
    ))(s)
}

#[cfg(test)]
mod test {
    use anyhow::Result;

    use super::*;

    #[test]
    fn parse_member_name() -> Result<()> {
        assert_eq!(member_name("abc034_fs3_u3")?.1, "abc034_fs3_u3");
        Ok(())
    }

    #[test]
    fn parse_member_name_should_fail_if_starting_with_underscore() {
        assert!(member_name("_invalid_identifier").is_err());
    }

    #[test]
    fn parse_member_name_should_fail_if_starting_with_number() {
        assert!(member_name("0invalid_identifier").is_err());
    }

    #[test]
    fn parse_message_name() -> Result<()> {
        assert_eq!(message_name("StdMsgs12")?.1, "StdMsgs12");
        Ok(())
    }

    #[test]
    fn parse_message_name_should_fail_if_starting_with_wrong_char() {
        assert!(message_name("aStdMsgs12").is_err());
    }

    #[test]
    fn parse_constant_name() -> Result<()> {
        assert_eq!(constant_name("C_O_N_STAN_T")?.1, "C_O_N_STAN_T");
        Ok(())
    }

    #[test]
    fn parse_constant_name_should_fail_if_starting_with_underscore() {
        assert!(constant_name("_C_O_N_STAN_Ta").is_err());
    }
}
