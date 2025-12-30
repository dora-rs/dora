//! ROS .msg file parser
//!
//! This module provides functionality to parse ROS .msg files and extract
//! message definitions for code generation and conversion.
//! Supports both ROS 1 and ROS 2 message formats (they are compatible).

use eyre::{Context, Result};
use std::path::{Path, PathBuf};

/// Represents a parsed ROS message field
#[derive(Debug, Clone)]
pub struct MessageField {
    pub field_type: String,
    pub name: String,
    pub default_value: Option<String>,
    pub is_array: bool,
    pub array_size: Option<usize>,
}

/// Represents a parsed ROS message definition
#[derive(Debug, Clone)]
pub struct MessageDefinition {
    pub package: String,
    pub name: String,
    pub fields: Vec<MessageField>,
    pub constants: Vec<MessageConstant>,
}

/// Represents a constant in a ROS message
#[derive(Debug, Clone)]
pub struct MessageConstant {
    pub constant_type: String,
    pub name: String,
    pub value: String,
}

/// Parse a ROS .msg file
pub fn parse_msg_file(path: &Path) -> Result<MessageDefinition> {
    let content = std::fs::read_to_string(path)
        .with_context(|| format!("Failed to read .msg file: {}", path.display()))?;

    // Extract package and message name from path
    let package = path
        .parent()
        .and_then(|p| p.parent())
        .and_then(|p| p.file_name())
        .and_then(|n| n.to_str())
        .unwrap_or("unknown")
        .to_string();

    let name = path
        .file_stem()
        .and_then(|n| n.to_str())
        .unwrap_or("Unknown")
        .to_string();

    let mut def = parse_msg_content(&content)?;
    def.package = package;
    def.name = name;
    Ok(def)
}

/// Parse ROS message content from a string
pub fn parse_msg_content(content: &str) -> Result<MessageDefinition> {
    let mut fields = Vec::new();
    let mut constants = Vec::new();

    for line in content.lines() {
        // Remove comments
        let (line, _) = line.split_once('#').unwrap_or((line, ""));
        let line = line.trim();

        // Skip empty lines
        if line.is_empty() {
            continue;
        }

        // Parse constants (format: TYPE NAME=VALUE)
        if let Some(equal_pos) = line.find('=') {
            let before = line[..equal_pos].trim();
            let after = line[equal_pos + 1..].trim();

            if let Some(space_pos) = before.rfind(char::is_whitespace) {
                let constant_type = before[..space_pos].trim().to_string();
                let name = before[space_pos..].trim().to_string();
                constants.push(MessageConstant {
                    constant_type,
                    name,
                    value: after.to_string(),
                });
            }
            continue;
        }

        // Parse fields (format: TYPE NAME or TYPE[] NAME for arrays, TYPE[N] for fixed arrays)
        if let Some(space_pos) = line.rfind(char::is_whitespace) {
            let field_type_part = line[..space_pos].trim();
            let name = line[space_pos..].trim().to_string();

            // Check for array syntax: TYPE[] or TYPE[N]
            let (field_type, is_array, array_size) = if field_type_part.ends_with(']') {
                if let Some(bracket_start) = field_type_part.rfind('[') {
                    let base_type = field_type_part[..bracket_start].trim();
                    let array_part = &field_type_part[bracket_start + 1..field_type_part.len() - 1];

                    if array_part.is_empty() {
                        // Unbounded array: TYPE[]
                        (base_type.to_string(), true, None)
                    } else {
                        // Fixed array: TYPE[N]
                        match array_part.parse::<usize>() {
                            Ok(size) => (base_type.to_string(), true, Some(size)),
                            Err(_) => (field_type_part.to_string(), false, None),
                        }
                    }
                } else {
                    (field_type_part.to_string(), false, None)
                }
            } else {
                (field_type_part.to_string(), false, None)
            };

            fields.push(MessageField {
                field_type,
                name,
                default_value: None,
                is_array,
                array_size,
            });
        }
    }

    Ok(MessageDefinition {
        package: "unknown".to_string(),
        name: "Unknown".to_string(),
        fields,
        constants,
    })
}

/// Find ROS message files in a workspace
/// Supports both ROS 1 (devel/share) and ROS 2 (install/share) layouts
pub fn find_msg_file(workspace: &Path, package: &str, message: &str) -> Result<PathBuf> {
    // Try ROS 2 layout first (install/share)
    let msg_path = workspace
        .join("install")
        .join(package)
        .join("share")
        .join(package)
        .join("msg")
        .join(format!("{}.msg", message));

    if msg_path.exists() {
        return Ok(msg_path);
    }

    // Try ROS 1 layout (devel/share or share)
    let msg_path = workspace
        .join("devel")
        .join("share")
        .join(package)
        .join("msg")
        .join(format!("{}.msg", message));

    if msg_path.exists() {
        return Ok(msg_path);
    }

    // Try direct share layout
    let msg_path = workspace
        .join("share")
        .join(package)
        .join("msg")
        .join(format!("{}.msg", message));

    if msg_path.exists() {
        return Ok(msg_path);
    }

    eyre::bail!(
        "Message file not found for {}/{} in workspace: {}",
        package,
        message,
        workspace.display()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_msg() {
        let content = r#"
# This is a comment
int32 x
float64 y
string name
"#;

        let msg = parse_msg_content(content).unwrap();
        assert_eq!(msg.fields.len(), 3);
        assert_eq!(msg.fields[0].name, "x");
        assert_eq!(msg.fields[0].field_type, "int32");
    }

    #[test]
    fn test_parse_with_constants() {
        let content = r#"
int32 FOO=42
string BAR="hello"
float64 PI=3.14159
int32 x
"#;

        let msg = parse_msg_content(content).unwrap();
        assert_eq!(msg.constants.len(), 3);
        assert_eq!(msg.fields.len(), 1);
    }
}
