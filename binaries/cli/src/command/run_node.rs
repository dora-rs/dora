//! The `dora run-node` command allows running individual nodes without creating a full dataflow.
//!
//! This command simplifies node development by allowing developers to test nodes
//! in isolation with custom inputs. It supports Python, Rust, and C/C++ nodes.

use super::Executable;
use eyre::{Context, Result};
use std::{path::PathBuf, process::Command};
use serde_yaml;
use std::collections::HashMap;

#[derive(Debug, clap::Args)]
/// Run a node in isolation.
///
/// Run a single node with custom inputs for testing and development.
/// Supports Python (.py), Rust binaries, and C/C++ shared libraries.
pub struct RunNode {
    /// Path to the node file
    #[clap(value_name = "NODE_PATH")]
    node_path: PathBuf,
    
    /// Path to input data file (YAML)
    #[clap(long, value_name = "INPUT_FILE")]
    input: Option<PathBuf>,
    
    /// Use UV to run Python nodes
    #[clap(long, action)]
    uv: bool,
}

impl Executable for RunNode {
    fn execute(self) -> Result<()> {
        run_node(self.node_path, self.input, self.uv)
    }
}

#[derive(serde::Deserialize, Debug)]
struct DataflowDescriptor {
    nodes: Vec<NodeDefinition>,
}

#[derive(serde::Deserialize, Debug)]
struct NodeDefinition {
    id: String,
    #[serde(default)]
    inputs: HashMap<String, serde_yaml::Value>,
    #[serde(default)]
    outputs: Vec<String>,
    #[serde(default)]
    env: HashMap<String, String>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum NodeType {
    Python,
    RustBinary,
    CSharedLibrary,
    Unknown,
}

fn detect_node_type(node_path: &PathBuf) -> NodeType {
    if let (Some(extension), Some(file_name)) = (node_path.extension(), node_path.file_name()) {
        match extension.to_str() {
            Some("py") => NodeType::Python,
            Some("so") | Some("dylib") | Some("dll") => NodeType::CSharedLibrary,
            _ => {
                // For files without extensions or with uncommon extensions,
                // check if it's an executable binary
                if cfg!(unix) {
                    // On Unix-like systems, check if it's executable
                    use std::os::unix::fs::MetadataExt;
                    if let Ok(metadata) = std::fs::metadata(node_path) {
                        if metadata.mode() & 0o111 != 0 {
                            NodeType::RustBinary
                        } else {
                            NodeType::Unknown
                        }
                    } else {
                        NodeType::Unknown
                    }
                } else {
                    // On other systems, assume unknown
                    NodeType::Unknown
                }
            }
        }
    } else if let Some(file_name) = node_path.file_name() {
        // Files without extensions - check if executable on Unix
        if cfg!(unix) {
            use std::os::unix::fs::MetadataExt;
            if let Ok(metadata) = std::fs::metadata(node_path) {
                if metadata.mode() & 0o111 != 0 {
                    NodeType::RustBinary
                } else {
                    NodeType::Unknown
                }
            } else {
                NodeType::Unknown
            }
        } else {
            NodeType::Unknown
        }
    } else {
        NodeType::Unknown
    }
}

fn run_node(
    node_path: PathBuf,
    input_file: Option<PathBuf>,
    uv: bool,
) -> Result<()> {
    println!("Running node: {:?}", node_path);
    
    // Detect node type
    let node_type = detect_node_type(&node_path);
    println!("Detected node type: {:?}", node_type);
    
    // Load inputs from file if provided
    let test_inputs = if let Some(input_path) = &input_file {
        println!("Using input file: {:?}", input_path);
        let input_content = std::fs::read_to_string(input_path)
            .context("Failed to read input file")?;
        
        // Try to parse as full dataflow descriptor first
        match serde_yaml::from_str::<DataflowDescriptor>(&input_content) {
            Ok(descriptor) => {
                // Extract inputs from the first node in the dataflow
                if let Some(first_node) = descriptor.nodes.first() {
                    println!("Using inputs from node: {}", first_node.id);
                    // Convert node inputs to test input format
                    let mut test_inputs = Vec::new();
                    for (input_id, value) in &first_node.inputs {
                        test_inputs.push(serde_yaml::Value::Mapping({
                            let mut map = serde_yaml::Mapping::new();
                            map.insert(serde_yaml::Value::String("type".to_string()), serde_yaml::Value::String("INPUT".to_string()));
                            map.insert(serde_yaml::Value::String("id".to_string()), serde_yaml::Value::String(input_id.clone()));
                            map.insert(serde_yaml::Value::String("value".to_string()), value.clone());
                            map.insert(serde_yaml::Value::String("metadata".to_string()), serde_yaml::Value::Mapping(serde_yaml::Mapping::new()));
                            map
                        }));
                    }
                    test_inputs
                } else {
                    Vec::new()
                }
            }
            Err(_) => {
                // Fall back to simple array format
                serde_yaml::from_str(&input_content)
                    .context("Failed to parse input file as YAML")?
            }
        }
    } else {
        Vec::new()
    };
    
    // Convert inputs to YAML string for environment variable
    let inputs_yaml = serde_yaml::to_string(&test_inputs)
        .context("Failed to serialize inputs to YAML")?;
    
    // Set up environment variables for the node
    let mut cmd = match node_type {
        NodeType::Python => {
            if uv {
                let mut cmd = Command::new("uv");
                cmd.arg("run");
                cmd.arg("python");
                cmd
            } else {
                Command::new("python")
            }
        },
        NodeType::RustBinary | NodeType::CSharedLibrary => {
            // For compiled nodes, execute directly
            Command::new(&node_path)
        },
        NodeType::Unknown => {
            // Default to Python for backward compatibility
            if uv {
                let mut cmd = Command::new("uv");
                cmd.arg("run");
                cmd.arg("python");
                cmd
            } else {
                Command::new("python")
            }
        }
    };
    
    // For Python nodes, we need to pass the script path
    if node_type == NodeType::Python {
        cmd.arg(&node_path);
    }
    
    // Set environment variables for testing
    cmd.env("DORA_TEST_INPUTS", inputs_yaml);
    cmd.env("DORA_TEST_MODE", "true");
    
    // Run the node
    println!("Executing node...");
    let output = cmd.output()
        .context("Failed to execute node")?;
    
    // Print stdout and stderr
    if !output.stdout.is_empty() {
        println!("Node stdout:");
        println!("{}", String::from_utf8_lossy(&output.stdout));
    }
    
    if !output.stderr.is_empty() {
        eprintln!("Node stderr:");
        eprintln!("{}", String::from_utf8_lossy(&output.stderr));
    }
    
    // Check exit status
    if output.status.success() {
        println!("Node executed successfully!");
        Ok(())
    } else {
        eyre::bail!("Node execution failed with exit code: {:?}", output.status.code());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn test_run_node_command_structure() {
        // Test that we can create a RunNode command struct
        let node_path = PathBuf::from("test_node.py");
        let cmd = RunNode {
            node_path,
            input: None,
            uv: false,
        };
        assert_eq!(cmd.node_path, PathBuf::from("test_node.py"));
    }
    
    #[test]
    fn test_node_type_detection() {
        // Test Python file detection
        let py_path = PathBuf::from("test_node.py");
        assert_eq!(detect_node_type(&py_path), NodeType::Python);
        
        // Test shared library detection (Unix)
        if cfg!(unix) {
            let so_path = PathBuf::from("test_node.so");
            assert_eq!(detect_node_type(&so_path), NodeType::CSharedLibrary);
        }
        
        // Test unknown extension
        let unknown_path = PathBuf::from("test_node.xyz");
        // This will depend on whether the file exists and is executable
        let _ = detect_node_type(&unknown_path);
    }
}