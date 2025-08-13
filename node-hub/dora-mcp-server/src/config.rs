use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use figment::Figment;
use figment::providers::{Env, Format, Json, Toml, Yaml};
use rmcp::model::{JsonObject, ToolAnnotations};
use serde::{Deserialize, Serialize};

pub static CONFIG: OnceLock<Config> = OnceLock::new();

fn figment_from_path<P: AsRef<Path>>(path: P) -> Figment {
    let ext = path
        .as_ref()
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or_default();
    match ext {
        "yaml" | "yml" => Figment::new().merge(Yaml::file(path)),
        "json" => Figment::new().merge(Json::file(path)),
        "toml" => Figment::new().merge(Toml::file(path)),
        _ => panic!("Unsupported config file format: {ext}"),
    }
}
pub fn init() {
    let config_file = Env::var("CONFIG").unwrap_or("config.toml".into());
    let config_path = PathBuf::from(config_file);
    if !config_path.exists() {
        eprintln!("Config file not found at: {}", config_path.display());
        std::process::exit(1);
    }

    let raw_config = figment_from_path(config_path);
    let conf = match raw_config.extract::<Config>() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("It looks like your config is invalid. The following error occurred: {e}");
            std::process::exit(1);
        }
    };

    CONFIG.set(conf).expect("config should be set");
}
pub fn get() -> &'static Config {
    CONFIG.get().unwrap()
}

#[derive(Clone, Debug, Deserialize)]
pub struct Config {
    #[serde(default = "default_listen_addr")]
    pub listen_addr: String,

    #[serde(default = "default_endpoint")]
    pub endpoint: Option<String>,

    pub name: String,
    pub version: String,

    pub mcp_tools: Vec<McpToolConfig>,
}
fn default_listen_addr() -> String {
    "0.0.0.0:8008".to_owned()
}
fn default_endpoint() -> Option<String> {
    Some("mcp".to_owned())
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct McpToolConfig {
    /// Unique identifier for the tool
    pub name: String,
    /// Optional human-readable name of the tool for display purposes
    #[serde(skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    /// Human-readable description of functionality
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// A JSON Schema object defining the expected parameters for the tool
    pub input_schema: InputSchema,
    #[serde(skip_serializing_if = "Option::is_none")]
    /// Optional properties describing tool behavior
    pub annotations: Option<ToolAnnotations>,

    pub output: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum InputSchema {
    Object(JsonObject),
    FilePath(String),
}

impl InputSchema {
    pub fn schema(&self) -> JsonObject {
        match self {
            InputSchema::Object(obj) => obj.clone(),
            InputSchema::FilePath(path) => figment_from_path(path)
                .extract::<JsonObject>()
                .expect("should read input schema from file"),
        }
    }
}
