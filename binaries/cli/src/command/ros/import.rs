use clap::Args;
use eyre::Context;
use std::path::PathBuf;

use crate::command::{Executable, default_tracing};

/// Import ROS message types and generate Dora bindings.
///
/// Examples:
///
/// Import common ROS message types:
///   dora ros import sensor_msgs/Image geometry_msgs/Twist nav_msgs/Path
///
/// Import from a specific ROS workspace:
///   dora ros import sensor_msgs/Image --ros-workspace /opt/ros/noetic
///
/// Import custom message types:
///   dora ros import my_package/MyMessage --msg-path ./msg
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Import {
    /// ROS message types to import (format: package_name/MessageName)
    #[clap(required = true, value_name = "MESSAGE_TYPE")]
    pub message_types: Vec<String>,

    /// ROS workspace path (e.g., /opt/ros/noetic for ROS 1, /opt/ros/humble for ROS 2)
    #[clap(long, value_name = "PATH")]
    pub ros_workspace: Option<PathBuf>,

    /// Path to custom .msg files directory
    #[clap(long, value_name = "PATH")]
    pub msg_path: Option<PathBuf>,

    /// Output directory for generated code
    #[clap(long, value_name = "PATH", default_value = "src/types/ros")]
    pub output: PathBuf,

    /// ROS version (1 or 2)
    #[clap(long, value_name = "VERSION", default_value = "2")]
    pub ros_version: String,
}

impl Executable for Import {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        import_ros_messages(self)
    }
}

fn import_ros_messages(args: Import) -> eyre::Result<()> {
    use dora_ros_compat::message_parser;
    use std::fs;
    use std::path::Path;

    println!("Importing ROS message types: {:?}", args.message_types);
    println!("ROS Version: {}", args.ros_version);
    
    // Create output directory if it doesn't exist
    fs::create_dir_all(&args.output)
        .with_context(|| format!("Failed to create output directory: {}", args.output.display()))?;

    let mut imported_count = 0;
    let mut errors = Vec::new();

    for message_type in &args.message_types {
        // Parse message type (format: package/MessageName)
        let parts: Vec<&str> = message_type.split('/').collect();
        if parts.len() != 2 {
            errors.push(format!("Invalid message type format: {}. Expected: package/MessageName", message_type));
            continue;
        }
        
        let package = parts[0];
        let message = parts[1];

        match import_single_message(
            package,
            message,
            &args.ros_workspace,
            &args.msg_path,
            &args.output,
            &args.ros_version,
        ) {
            Ok(_) => {
                println!("✓ Imported {}/{}", package, message);
                imported_count += 1;
            }
            Err(e) => {
                let error_msg = format!("Failed to import {}/{}: {}", package, message, e);
                eprintln!("✗ {}", error_msg);
                errors.push(error_msg);
            }
        }
    }

    if !errors.is_empty() {
        eprintln!("\nErrors encountered:");
        for error in &errors {
            eprintln!("  - {}", error);
        }
    }

    println!("\n✓ Successfully imported {} ROS message types", imported_count);
    println!("✓ Generated code in {}", args.output.display());
    
    if imported_count == 0 {
        eyre::bail!("No messages were successfully imported");
    }

    Ok(())
}

fn import_single_message(
    package: &str,
    message: &str,
    ros_workspace: &Option<std::path::PathBuf>,
    msg_path: &Option<std::path::PathBuf>,
    output_dir: &std::path::Path,
    _ros_version: &str,
) -> eyre::Result<()> {
    use dora_ros_compat::message_parser;
    use std::fs;
    use std::io::Write;

    // Find the .msg file
    let msg_file = if let Some(custom_path) = msg_path {
        custom_path.join(format!("{}.msg", message))
    } else if let Some(workspace) = ros_workspace {
        dora_ros_compat::message_parser::find_msg_file(workspace, package, message)?
    } else {
        // Try to find in standard ROS locations
        let ros_distro = std::env::var("ROS_DISTRO").unwrap_or_else(|_| "noetic".to_string());
        let default_workspace = std::path::PathBuf::from(format!("/opt/ros/{}", ros_distro));
        dora_ros_compat::message_parser::find_msg_file(&default_workspace, package, message)
            .or_else(|_| {
                // Try ROS 2 location
                let ros2_distro = std::env::var("ROS_DISTRO").unwrap_or_else(|_| "humble".to_string());
                let ros2_workspace = std::path::PathBuf::from(format!("/opt/ros/{}", ros2_distro));
                dora_ros_compat::message_parser::find_msg_file(&ros2_workspace, package, message)
            })?
    };

    // Parse the message file
    let msg_def = message_parser::parse_msg_file(&msg_file)
        .with_context(|| format!("Failed to parse message file: {}", msg_file.display()))?;

    // Generate Rust code
    let rust_code = generate_rust_binding(&msg_def)?;
    let rust_dir = output_dir.join("rust");
    fs::create_dir_all(&rust_dir)?;
    let rust_file = rust_dir.join(format!("{}_{}.rs", package, message));
    let mut file = fs::File::create(&rust_file)?;
    file.write_all(rust_code.as_bytes())?;

    // Generate Python stub (placeholder for now)
    let python_code = generate_python_stub(&msg_def)?;
    let python_dir = output_dir.join("python");
    fs::create_dir_all(&python_dir)?;
    let python_file = python_dir.join(format!("{}_{}.py", package, message));
    let mut file = fs::File::create(&python_file)?;
    file.write_all(python_code.as_bytes())?;

    Ok(())
}

fn generate_rust_binding(msg_def: &dora_ros_compat::message_parser::MessageDefinition) -> eyre::Result<String> {
    let mut code = String::new();
    
    code.push_str(&format!("//! Auto-generated Rust binding for {}/{}\n", msg_def.package, msg_def.name));
    code.push_str("//! Generated by dora ros import\n\n");
    code.push_str("use arrow::array::ArrayRef;\n");
    code.push_str("use eyre::Result;\n\n");
    
    code.push_str(&format!("pub struct {} {{\n", msg_def.name));
    for field in &msg_def.fields {
        let rust_type = ros_type_to_rust(&field.field_type, field.is_array, field.array_size);
        code.push_str(&format!("    pub {}: {},\n", field.name, rust_type));
    }
    code.push_str("}\n\n");
    
    code.push_str(&format!("impl {} {{\n", msg_def.name));
    code.push_str("    pub fn to_dora(&self) -> Result<ArrayRef> {\n");
    code.push_str("        // TODO: Implement conversion to Arrow format\n");
    code.push_str("        todo!(\"Conversion not yet implemented\")\n");
    code.push_str("    }\n");
    code.push_str("}\n");
    
    Ok(code)
}

fn generate_python_stub(msg_def: &dora_ros_compat::message_parser::MessageDefinition) -> eyre::Result<String> {
    let mut code = String::new();
    
    code.push_str(&format!("\"\"\"Auto-generated Python binding for {}/{}\"\"\"\n", msg_def.package, msg_def.name));
    code.push_str("# Generated by dora ros import\n\n");
    code.push_str("from typing import Any\n");
    code.push_str("import pyarrow as pa\n\n");
    
    code.push_str(&format!("class {}:\n", msg_def.name));
    code.push_str(&format!("    \"\"\"ROS message type: {}/{}\"\"\"\n\n", msg_def.package, msg_def.name));
    code.push_str("    def __init__(self):\n");
    for field in &msg_def.fields {
        code.push_str(&format!("        self.{} = None\n", field.name));
    }
    code.push_str("\n");
    code.push_str("    def to_dora(self) -> pa.Array:\n");
    code.push_str("        \"\"\"Convert to Dora Arrow format\"\"\"\n");
    code.push_str("        # TODO: Implement conversion\n");
    code.push_str("        raise NotImplementedError(\"Conversion not yet implemented\")\n");
    
    Ok(code)
}

fn ros_type_to_rust(ros_type: &str, is_array: bool, array_size: Option<usize>) -> String {
    let base_type: String = match ros_type {
        "bool" => "bool".to_string(),
        "int8" => "i8".to_string(),
        "int16" => "i16".to_string(),
        "int32" => "i32".to_string(),
        "int64" => "i64".to_string(),
        "uint8" => "u8".to_string(),
        "uint16" => "u16".to_string(),
        "uint32" => "u32".to_string(),
        "uint64" => "u64".to_string(),
        "float32" => "f32".to_string(),
        "float64" => "f64".to_string(),
        "string" => "String".to_string(),
        "time" => "std::time::SystemTime".to_string(),
        "duration" => "std::time::Duration".to_string(),
        _ => {
            // Assume it's a custom message type
            if ros_type.contains('/') {
                // Namespaced type: package/Message
                let parts: Vec<&str> = ros_type.split('/').collect();
                format!("super::{}::{}", parts[0], parts[1])
            } else {
                // Local type reference
                format!("super::{}", ros_type)
            }
        }
    };

    if is_array {
        if let Some(size) = array_size {
            format!("[{}; {}]", base_type, size)
        } else {
            format!("Vec<{}>", base_type)
        }
    } else {
        base_type.to_string()
    }
}

