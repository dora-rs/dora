use std::{fs, path::Path};

use anyhow::{Context, Result};

use super::{constant::constant_def, member::member_def};
use crate::types::Message;

fn split_once(s: &'_ str, pat: char) -> (&'_ str, Option<&'_ str>) {
    let mut items = s.splitn(2, pat);
    (items.next().unwrap(), items.next())
}

/// Byte index of the first occurrence of `target` that lies outside any
/// single- or double-quoted string, or `None` if there is none.
///
/// Quoting follows the ROS 2 message-format string literals parsed by
/// [`super::literal`]: a `"` or `'` opens a string that runs until the matching
/// closing quote, and a backslash escapes the next byte (so `\"` does not close
/// a double-quoted string). `target`, the quote characters and the backslash
/// are all ASCII, and multi-byte UTF-8 continuation bytes never collide with
/// them, so the returned index is always on a `char` boundary.
fn find_unquoted(s: &str, target: u8) -> Option<usize> {
    let bytes = s.as_bytes();
    let mut quote: Option<u8> = None;
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i];
        match quote {
            Some(q) => {
                if c == b'\\' {
                    i += 1; // skip the escaped byte
                } else if c == q {
                    quote = None;
                }
            }
            None => {
                if c == target {
                    return Some(i);
                }
                if c == b'"' || c == b'\'' {
                    quote = Some(c);
                }
            }
        }
        i += 1;
    }
    None
}

/// Whether `type_token` names a ROS `string`/`wstring` type in any of its forms
/// (`string`, `wstring`, the bounded `string<=N` / `wstring<=N`, and the array /
/// sequence variants such as `string[3]` or `wstring<=5[]`).
///
/// Only these lead to a value whose text may legally contain a `#` outside of
/// quotes, so the check is deliberately exact: a custom message type whose name
/// merely starts with `string` (e.g. `stringlike`) must not match.
fn is_string_type(type_token: &str) -> bool {
    ["string", "wstring"].iter().any(|base| {
        type_token
            .strip_prefix(base)
            .is_some_and(|rest| rest.is_empty() || rest.starts_with('<') || rest.starts_with('['))
    })
}

/// Strip a trailing `# comment` from a definition line, respecting ROS 2 string
/// semantics so that a `#` that is part of a value is not mistaken for a comment.
///
/// Two cases the naive "cut at the first `#`" does wrong:
///
/// 1. A `#` inside a quoted string is part of the string, e.g. the default of
///    `string greeting "hi # there"`. Cutting there truncates the value and, for
///    a double-quoted value, leaves an unterminated quote that fails the whole
///    file.
/// 2. A string *constant*'s value runs to the end of the line and may contain an
///    unquoted `#`, e.g. `string PATTERN=a#b`. The ROS 2 message format does not
///    treat `#` as a comment delimiter inside a string constant value, so the
///    line must be kept intact.
///
/// Non-string lines (and string *member* lines without a default value) keep the
/// ordinary rule: the first `#` outside any quoted string begins a comment.
fn strip_comment(line: &str) -> &str {
    let trimmed = line.trim_start();
    let type_token = trimmed.split([' ', '\t']).next().unwrap_or("");
    // A string constant is `string NAME=VALUE`: a string-typed line with an `=`
    // outside quotes after the type token. Its value extends to end-of-line, so
    // leave the line untouched. (`string<=N` keeps its own `=` inside the type
    // token, hence the search starts after the first whitespace run.)
    if is_string_type(type_token)
        && let Some((_, rest)) = trimmed.split_once([' ', '\t'])
        && find_unquoted(rest, b'=').is_some()
    {
        return line;
    }
    find_unquoted(line, b'#').map_or(line, |idx| &line[..idx])
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
        //
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
    fn string_constant_value_may_contain_hash() {
        // A `#` inside a string constant value is part of the value, not the
        // start of a comment. Cutting at the first `#` silently truncated the
        // value to "a".
        let msg = parse_message_string("pkg", "Msg", "string PATTERN=a#b").unwrap();
        assert_eq!(msg.constants.len(), 1, "expected a constant, got {msg:?}");
        assert!(msg.members.is_empty());
        assert_eq!(msg.constants[0].value, vec!["a#b".to_string()]);
    }

    #[test]
    fn quoted_string_value_containing_hash_is_not_truncated() {
        // A `#` inside a double-quoted value must not be treated as a comment;
        // truncating there left an unterminated quote that failed the whole file.
        let msg = parse_message_string("pkg", "Msg", r#"string GREETING="a # b""#).unwrap();
        assert_eq!(msg.constants.len(), 1, "expected a constant, got {msg:?}");
        assert_eq!(msg.constants[0].value, vec!["a # b".to_string()]);

        // The same for a single-quoted member default.
        let member = parse_message_string("pkg", "Msg", "string greeting 'hi # there'").unwrap();
        assert_eq!(member.members.len(), 1, "expected a member, got {member:?}");
        assert_eq!(
            member.members[0].default,
            Some(vec!["hi # there".to_string()])
        );
    }

    #[test]
    fn trailing_comment_is_still_stripped() {
        // Ordinary trailing comments on non-string-constant lines must still be
        // removed.
        let msg = parse_message_string("pkg", "Msg", "int32 X=5 # the answer minus 37").unwrap();
        assert_eq!(msg.constants.len(), 1, "expected a constant, got {msg:?}");
        assert_eq!(msg.constants[0].value, vec!["5".to_string()]);

        let member = parse_message_string("pkg", "Msg", "int32 count # a comment").unwrap();
        assert_eq!(member.members.len(), 1, "expected a member, got {member:?}");
        assert_eq!(member.members[0].name, "count");
        assert!(member.members[0].default.is_none());

        // A whole-line comment is ignored entirely.
        let empty = parse_message_string("pkg", "Msg", "# just a comment").unwrap();
        assert!(empty.constants.is_empty() && empty.members.is_empty());
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
}
