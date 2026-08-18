use nom::{
    IResult, Parser,
    branch::alt,
    character::complete::{char, one_of, satisfy},
    combinator::{opt, recognize},
    multi::{many0_count, many1, separated_list0, separated_list1},
    sequence::pair,
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
    recognize((
        loweralpha,
        opt(char('_')),
        separated_list1(char('_'), many1(alt((loweralpha, numeric)))),
    ))
    .parse(s)
}

pub fn member_name(s: &str) -> IResult<&str, &str> {
    recognize((
        loweralpha,
        opt(char('_')),
        separated_list0(char('_'), many1(alt((loweralpha, numeric)))),
    ))
    .parse(s)
}

pub fn message_name(s: &str) -> IResult<&str, &str> {
    // Note: consume the trailing alphanumerics char by char instead of using
    // `alphanumeric0`. In nom 8.0.0, `&str`'s `split_at_position_complete`
    // returns a remainder pointing at the *start* of the input when the parser
    // consumes everything, which makes the offset-based `recognize` return an
    // empty slice.
    recognize(pair(
        upperalpha,
        many0_count(satisfy(|c| c.is_ascii_alphanumeric())),
    ))
    .parse(s)
}

pub fn constant_name(s: &str) -> IResult<&str, &str> {
    recognize(separated_list1(
        char('_'),
        many1(alt((upperalpha, numeric))),
    ))
    .parse(s)
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
