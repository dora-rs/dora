use std::{fs, path::Path};

use anyhow::{Context, Result};

use super::{constant::constant_def, member::member_def};
use crate::types::Message;

fn split_once(s: &'_ str, pat: char) -> (&'_ str, Option<&'_ str>) {
    let mut items = s.splitn(2, pat);
    (items.next().unwrap(), items.next())
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
        let (line, _) = split_once(line, '#');
        let line = line.trim();
        if line.is_empty() {
            continue;
        }

        let (_, rest) = split_once(line, ' ');

        match rest.unwrap().find('=') {
            Some(_) => constants.push(constant_def(line)?),
            None => members.push(member_def(line)?),
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
}
