use std::{fs, path::Path};

use anyhow::{Context, Result};

use super::{constant::constant_def, member::member_def};
use crate::types::Message;

fn split_once(s: &'_ str, pat: char) -> (&'_ str, Option<&'_ str>) {
    let mut items = s.splitn(2, pat);
    (items.next().unwrap(), items.next())
}

fn strip_comment(s: &str) -> &str {
    let mut in_quotes = false;
    for (i, c) in s.char_indices() {
        if c == '"' {
            in_quotes = !in_quotes;
        } else if c == '#' && !in_quotes {
            //if is the first character of the line or the previous character is a blackspace
            if i == 0 || s.as_bytes()[i - 1].is_ascii_whitespace() {
                return &s[..i];
            }
        }
    }
    s
}
pub fn parse_message_file<P: AsRef<Path>>(pkg_name: &str, interface_file: P) -> Result<Message> {
    parse_message_string(
        pkg_name,
        interface_file
            .as_ref()
            .file_stem()
            .unwrap()
            .to_str()
            .unwrap(),
        fs::read_to_string(interface_file.as_ref())?.as_str(),
    )
    .with_context(|| format!("Parse file error: {}", interface_file.as_ref().display()))
}

pub fn parse_message_string(
    pkg_name: &str,
    msg_name: &str,
    message_string: &str,
) -> Result<Message> {
    let mut members = vec![];
    let mut constants = vec![];

    for line in message_string.lines() {
        let line = strip_comment(line);
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        let (_, rest) = split_once(line, ' ');

        // A constant line is `TYPE NAME = VALUE`: the `=` binds the name
        // (optionally with surrounding spaces, which `constant_def` accepts). A
        // member line is `TYPE name [default]`, and a *string* default may
        // itself contain `=` (e.g. `string url "http://h?a=b"`). Classifying by
        // "does the remainder contain `=` anywhere" misroutes such a member to
        // `constant_def`, which then rejects the lowercase field name and fails
        // the whole message file. Instead, look only at the name position: the
        // first non-space, non-`=` run after the type is the name, so the line
        // is a constant iff the first character after that name (skipping
        // spaces) is `=`.
        //parse_message_string
        // rest is None when the line has no space (e.g. tab-separated or
        // single-token); that is never a constant, so it falls through to
        // `member_def` rather than panicking.
        let is_constant = rest.is_some_and(|r| {
            r.trim_start()
                .trim_start_matches(|c: char| !c.is_whitespace() && c != '=')
                .trim_start()
                .starts_with('=')
        });
        if is_constant {
            constants.push(constant_def(line)?);
        } else {
            members.push(member_def(line)?);
        }
    }

    Ok(Message {
        package: pkg_name.into(),
        name: msg_name.into(),
        members,
        constants,
    })
}

#[cfg(test)]
mod test {
    use std::path::PathBuf;

    use super::*;
    use crate::types::{primitives::*, sequences::*, *};

    #[test]
    fn test_split_once() {
        assert_eq!(split_once("abc", 'b'), ("a", Some("c")));
        assert_eq!(split_once("abc", 'c'), ("ab", Some("")));
        assert_eq!(split_once("abc", 'd'), ("abc", None));
    }

    fn parse_msg_def(msg_name: &str) -> Result<Message> {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join(format!("test_msgs/msg/{}.msg", msg_name));
        parse_message_file("test_msgs", path)
    }

    #[test]
    fn parse_arrays() -> Result<()> {
        let message = parse_msg_def("Arrays")?;

        assert_eq!(message.package, "test_msgs".to_string());
        assert_eq!(message.name, "Arrays".to_string());
        assert_eq!(message.members[0].name, "bool_values".to_string());
        assert_eq!(
            message.members[0].r#type,
            MemberType::Array(Array {
                value_type: BasicType::Bool.into(),
                size: 3,
            })
        );

        Ok(())
    }

    #[test]
    fn parse_basic_types() -> Result<()> {
        let result = parse_msg_def("BasicTypes")?;

        assert_eq!(result.members[0].name, "bool_value".to_string());
        assert_eq!(result.members[0].r#type, BasicType::Bool.into());
        assert_eq!(result.members[0].default, None);

        Ok(())
    }

    #[test]
    fn parse_bounded_sequences() -> Result<()> {
        let _result = parse_msg_def("BoundedSequences")?;
        Ok(())
    }

    #[test]
    fn parse_constants() -> Result<()> {
        let _result = parse_msg_def("Constants")?;
        Ok(())
    }

    #[test]
    fn parse_defaults() -> Result<()> {
        let _result = parse_msg_def("Defaults")?;
        Ok(())
    }

    #[test]
    fn parse_empty() -> Result<()> {
        let _result = parse_msg_def("Empty")?;
        Ok(())
    }

    #[test]
    fn parse_multi_nested() -> Result<()> {
        let _result = parse_msg_def("MultiNested")?;
        Ok(())
    }

    #[test]
    fn parse_nested() -> Result<()> {
        let _result = parse_msg_def("Nested")?;
        Ok(())
    }

    #[test]
    fn parse_strings() -> Result<()> {
        let _result = parse_msg_def("Strings")?;
        Ok(())
    }

    #[test]
    fn parse_unbounded_sequences() -> Result<()> {
        let _result = parse_msg_def("UnboundedSequences")?;
        Ok(())
    }

    #[test]
    fn parse_wstrings() -> Result<()> {
        let _result = parse_msg_def("WStrings")?;
        Ok(())
    }

    #[test]
    fn parse_message_string_tab_separated_field() -> Result<()> {
        // Tab-separated lines must produce a member, not panic.
        let msg = parse_message_string("pkg", "Msg", "int8\tfoo")?;
        assert_eq!(msg.members.len(), 1);
        assert_eq!(msg.members[0].name, "foo");
        Ok(())
    }

    #[test]
    fn parse_message_string_single_token_line_returns_error() {
        // A single-token line with no space is malformed and must return an
        // error, not a panic.
        let result = parse_message_string("pkg", "Msg", "garbage");
        assert!(result.is_err());
    }

    #[test]
    fn string_member_default_containing_eq_is_a_member_not_a_constant() {
        // A string field whose default value contains `=` (e.g. a URL query
        // string) must be parsed as a member. Classifying it as a constant
        // (because the line contains `=` anywhere) rejects the lowercase field
        // name and fails the whole message.
        let msg = parse_message_string("pkg", "Msg", "string url \"http://h?a=b\"").unwrap();
        assert_eq!(msg.members.len(), 1, "expected a member, got {msg:?}");
        assert!(msg.constants.is_empty());
        assert_eq!(msg.members[0].name, "url");
    }

    #[test]
    fn constant_line_is_still_a_constant() {
        // Regression guard the other way: a genuine constant must still be
        // classified as one, both tightly written and with spaces around `=`.
        let msg = parse_message_string("pkg", "Msg", "int32 X=5").unwrap();
        assert_eq!(msg.constants.len(), 1, "expected a constant, got {msg:?}");
        assert!(msg.members.is_empty());
        assert_eq!(msg.constants[0].name, "X");

        let spaced = parse_message_string("pkg", "Msg", "int32 Y = 7").unwrap();
        assert_eq!(
            spaced.constants.len(),
            1,
            "expected a constant, got {spaced:?}"
        );
        assert_eq!(spaced.constants[0].name, "Y");
    }
    #[test]
    fn hash_in_unquoted_string_value() {
        let msg = parse_message_string("pkg", "Msg", "string PATTERN=a#b").unwrap();
        assert_eq!(msg.constants[0].value, vec!["a#b".to_string()]);
    }

    #[test]
    fn hash_in_quoted_string_value() {
        let msg = parse_message_string("pkg", "Msg", r#"string GREETING="a # b""#).unwrap();
        assert_eq!(msg.constants[0].value, vec!["a # b".to_string()]);
    }
}
