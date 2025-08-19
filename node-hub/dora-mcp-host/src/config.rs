use std::{
    collections::HashMap,
    path::PathBuf,
    process::Stdio,
    sync::{Arc, OnceLock},
};
use tokio::sync::mpsc;

use figment::Figment;
use figment::providers::{Env, Format, Json, Toml, Yaml};
use outfox_openai::spec::ChatCompletionRequestMessage;
use rmcp::{RoleClient, ServiceExt, service::RunningService, transport::ConfigureCommandExt};
use serde::{Deserialize, Serialize};

use crate::client::{ChatClient, DeepseekClient, DoraClient, GeminiClient, OpenaiClient};
use crate::{ChatSession, ServerEvent, tool::ToolSet};

pub static CONFIG: OnceLock<Config> = OnceLock::new();

pub fn init() {
    let config_file = Env::var("CONFIG").unwrap_or("config.toml".into());
    let config_path = PathBuf::from(config_file);
    if !config_path.exists() {
        eprintln!("Config file not found at: {}", config_path.display());
        std::process::exit(1);
    }

    let raw_config = match config_path
        .extension()
        .unwrap_or_default()
        .to_str()
        .unwrap_or_default()
    {
        "yaml" | "yml" => Figment::new().merge(Yaml::file(config_path)),
        "json" => Figment::new().merge(Json::file(config_path)),
        "toml" => Figment::new().merge(Toml::file(config_path)),
        ext => {
            eprintln!("unsupported config file format: {ext:?}");
            std::process::exit(1);
        }
    };

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

    #[serde(default)]
    pub providers: Vec<ProviderConfig>,
    #[serde(default)]
    pub models: Vec<ModelConfig>,
    #[serde(default)]
    pub prompts: Vec<ChatCompletionRequestMessage>,

    pub mcp: Option<McpConfig>,
    // #[serde(default = "default_true")]
    // pub support_tool: bool,
}
fn default_listen_addr() -> String {
    "0.0.0.0:8008".to_owned()
}
fn default_endpoint() -> Option<String> {
    Some("v1".to_owned())
}

// fn default_true() -> bool {
//     true
// }

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ProviderConfig {
    Gemini(GeminiConfig),
    Deepseek(DeepseekConfig),
    Openai(OpenaiConfig),
    Dora(DoraConfig),
}
impl ProviderConfig {
    pub fn id(&self) -> &str {
        match self {
            ProviderConfig::Gemini(config) => &config.id,
            ProviderConfig::Deepseek(config) => &config.id,
            ProviderConfig::Openai(config) => &config.id,
            ProviderConfig::Dora(config) => &config.id,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct GeminiConfig {
    pub id: String,
    #[serde(default = "default_gemini_api_key")]
    pub api_key: String,
    #[serde(default = "default_gemini_api_url")]
    pub api_url: String,
    #[serde(default)]
    pub proxy: bool,
}
fn default_gemini_api_key() -> String {
    std::env::var("GEMINI_API_KEY").unwrap_or_default()
}
fn default_gemini_api_url() -> String {
    std::env::var("GEMINI_API_URL").unwrap_or_else(|_|"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash:generateContent".to_owned())
}

#[derive(Clone, Debug, Deserialize)]
pub struct DeepseekConfig {
    pub id: String,
    #[serde(default = "default_deepseek_api_key")]
    pub api_key: String,
    #[serde(default = "default_deepseek_api_url")]
    pub api_url: String,
    #[serde(default)]
    pub proxy: bool,
}
fn default_deepseek_api_key() -> String {
    std::env::var("DEEPSEEK_API_KEY").unwrap_or_default()
}
fn default_deepseek_api_url() -> String {
    std::env::var("DEEPSEEK_API_URL").unwrap_or_else(|_| "https://api.deepseek.com".to_owned())
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenaiConfig {
    pub id: String,
    #[serde(default = "default_openai_api_key")]
    pub api_key: String,
    #[serde(default = "default_openai_api_url")]
    pub api_url: String,
    #[serde(default)]
    pub proxy: bool,
}
fn default_openai_api_key() -> String {
    std::env::var("OPENAI_API_KEY").unwrap_or_default()
}
fn default_openai_api_url() -> String {
    std::env::var("OPENAI_API_URL")
        .unwrap_or_else(|_| "https://api.openai.com/v1/chat/completions".to_owned())
}

#[derive(Clone, Debug, Deserialize)]
pub struct DoraConfig {
    pub id: String,
    pub output: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "model", rename_all = "snake_case")]
pub struct ModelConfig {
    pub id: String,
    pub object: Option<String>,
    pub created: Option<u32>,
    pub owned_by: Option<String>,

    // #[serde(default)]
    // pub default: bool,
    pub route: ModelRouteConfig,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "route", rename_all = "snake_case")]
pub struct ModelRouteConfig {
    pub provider: String,
    pub model: Option<String>,
}

#[derive(Clone, Default, Debug, Deserialize)]
pub struct McpConfig {
    #[serde(default)]
    pub servers: Vec<McpServerConfig>,
}
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct McpServerConfig {
    pub name: String,
    #[serde(flatten)]
    pub transport: McpServerTransportConfig,
}
#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(tag = "protocol", rename_all = "snake_case")]
pub enum McpServerTransportConfig {
    Streamable {
        url: String,
    },
    Sse {
        url: String,
    },
    Stdio {
        command: String,
        #[serde(default)]
        args: Vec<String>,
        #[serde(default)]
        envs: HashMap<String, String>,
    },
}

impl McpServerTransportConfig {
    pub async fn start(&self) -> eyre::Result<RunningService<RoleClient, ()>> {
        let client = match self {
            McpServerTransportConfig::Streamable { url } => {
                for _ in 0..5 {
                    let transport =
                        rmcp::transport::StreamableHttpClientTransport::from_uri(url.to_string());
                    match ().serve(transport).await {
                        Ok(client) => return Ok(client),
                        Err(e) => {
                            println!("failed to start streamable transport: {e}");
                            tracing::warn!("failed to start streamable transport: {e}");
                            tokio::time::sleep(std::time::Duration::from_secs(2)).await;
                        }
                    }
                }
                eyre::bail!("failed to start streamable transport after 5 attempts");
            }
            McpServerTransportConfig::Sse { url } => {
                let transport =
                    rmcp::transport::sse_client::SseClientTransport::start(url.to_owned()).await?;
                ().serve(transport).await?
            }
            McpServerTransportConfig::Stdio {
                command,
                args,
                envs,
            } => {
                let transport = rmcp::transport::TokioChildProcess::new(
                    tokio::process::Command::new(command).configure(|cmd| {
                        cmd.args(args)
                            .envs(envs)
                            .stderr(Stdio::inherit())
                            .stdout(Stdio::inherit());
                    }),
                )?;
                ().serve(transport).await?
            }
        };
        Ok(client)
    }
}

impl Config {
    pub async fn create_mcp_clients(
        &self,
    ) -> eyre::Result<HashMap<String, RunningService<RoleClient, ()>>> {
        let mut clients = HashMap::new();

        if let Some(mcp_config) = &self.mcp {
            for server in &mcp_config.servers {
                let client = server.transport.start().await?;
                clients.insert(server.name.clone(), client);
            }
        }

        Ok(clients)
    }

    fn create_chat_clients(
        &self,
        server_events_tx: mpsc::Sender<ServerEvent>,
    ) -> HashMap<String, Arc<dyn ChatClient>> {
        let mut clients: HashMap<String, Arc<dyn ChatClient>> = HashMap::new();
        for provider in &self.providers {
            let client: Arc<dyn ChatClient> = match provider {
                ProviderConfig::Gemini(config) => Arc::new(GeminiClient::new(config)),
                ProviderConfig::Deepseek(config) => Arc::new(DeepseekClient::new(config)),
                ProviderConfig::Openai(config) => Arc::new(OpenaiClient::new(config)),
                ProviderConfig::Dora(config) => {
                    Arc::new(DoraClient::new(config, server_events_tx.clone()))
                }
            };
            clients.insert(provider.id().to_owned(), client);
        }
        clients
    }

    pub async fn create_session(
        &self,
        server_events_tx: mpsc::Sender<ServerEvent>,
    ) -> eyre::Result<ChatSession> {
        let mut tool_set = ToolSet::default();

        if self.mcp.is_some() {
            let mcp_clients = self.create_mcp_clients().await?;

            for (name, client) in mcp_clients.iter() {
                tracing::info!("load MCP tool: {name}");
                let server = client.peer().clone();
                let tools = crate::get_mcp_tools(server.clone()).await?;

                for tool in tools {
                    tool_set.add_tool(tool);
                }
            }
            tool_set.set_clients(mcp_clients);
        }

        Ok(ChatSession::new(
            self.create_chat_clients(server_events_tx),
            self.models.clone(),
            self.prompts.clone(),
            tool_set,
        ))
    }
}
