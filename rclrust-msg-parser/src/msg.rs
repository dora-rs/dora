use std::fs;
use std::path::Path;

use anyhow::{Context, Result};
use rclrust_msg_types::Message;

use crate::constant::constant_def;
use crate::member::member_def;

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
    use super::*;

    #[test]
    fn test_split_once() {
        assert_eq!(split_once("abc", 'b'), ("a", Some("c")));
        assert_eq!(split_once("abc", 'c'), ("ab", Some("")));
        assert_eq!(split_once("abc", 'd'), ("abc", None));
    }
}
