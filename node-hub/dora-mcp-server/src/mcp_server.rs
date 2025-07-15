use std::sync::Arc;

use futures::channel::oneshot;
use rmcp::model::JsonRpcRequest;
use rmcp::model::{
    CallToolResult, EmptyResult, Implementation, InitializeResult, JsonObject, ListToolsResult,
    ProtocolVersion, Request, ServerCapabilities, ServerResult, Tool,
};
use serde::Deserialize;
use tokio::sync::mpsc;

use crate::{Config, ServerEvent};

#[derive(Debug)]
pub struct McpServer {
    tools: Vec<McpTool>,
    server_info: Implementation,
}

#[derive(Deserialize, Debug)]
pub struct McpTool {
    pub node_id: String,
    #[serde(flatten)]
    pub inner: Tool,
}

impl McpServer {
    pub fn new(config: &Config) -> Self {
        let mut tools = Vec::new();
        for tool_config in &config.mcp_tools {
            let tool = Tool {
                name: tool_config.name.clone().into(),
                description: tool_config.description.clone().map(|s| s.into()),
                input_schema: Arc::new(tool_config.input_schema.schema()),
                annotations: tool_config.annotations.clone(),
            };
            tools.push(McpTool {
                inner: tool,
                node_id: tool_config.node_id.clone(),
            });
        }
        Self {
            tools,
            server_info: Implementation {
                name: config.name.clone(),
                version: config.version.clone(),
            },
        }
    }

    // pub fn tools(&self) -> Vec<&Tool> {
    //     self.tools.iter().map(|t| &t.inner).collect()
    // }

    // pub fn server_info(&self) -> &Implementation {
    //     &self.server_info
    // }

    pub async fn handle_ping(&self) -> eyre::Result<EmptyResult> {
        Ok(EmptyResult {})
    }
    pub async fn handle_initialize(&self) -> eyre::Result<InitializeResult> {
        Ok(InitializeResult {
            protocol_version: ProtocolVersion::V_2025_03_26,
            server_info: self.server_info.clone(),
            capabilities: ServerCapabilities {
                tools: Some(Default::default()),
                ..Default::default()
            },
            instructions: None,
        })
    }
    pub async fn handle_tools_list(&self) -> eyre::Result<ListToolsResult> {
        Ok(ListToolsResult {
            tools: self.tools.iter().map(|t| t.inner.clone()).collect(),
            next_cursor: None,
        })
    }

    pub async fn handle_tools_call(
        &self,
        params: JsonObject,
        request_tx: &mpsc::Sender<ServerEvent>,
    ) -> eyre::Result<CallToolResult> {
        let (tx, rx) = oneshot::channel();

        let Some(name) = params.get("name") else {
            return Err(eyre::eyre!("Tool name is required in parameters"));
        };
        let name = name
            .as_str()
            .ok_or_else(|| eyre::eyre!("Tool name must be a string"))?;
        let tool = self
            .tools
            .iter()
            .find(|t| t.inner.name == name)
            .ok_or_else(|| eyre::eyre!("Tool not found: {}", name))?;
        request_tx
            .send(ServerEvent::CallNode {
                node_id: tool.node_id.clone(),
                data: serde_json::to_string(&params).unwrap(),
                reply: tx,
            })
            .await?;

        let data: String = rx.await?;
        serde_json::from_str(&data)
            .map_err(|e| eyre::eyre!("Failed to parse call tool result: {e}"))
    }

    pub async fn handle_request(
        &self,
        rpc_request: JsonRpcRequest,
        server_events_tx: &mpsc::Sender<ServerEvent>,
    ) -> eyre::Result<ServerResult> {
        let Request { method, params, .. } = rpc_request.request;
        match method.as_str() {
            "ping" => self.handle_ping().await.map(ServerResult::EmptyResult),
            "initialize" => self
                .handle_initialize()
                .await
                .map(ServerResult::InitializeResult),
            "tools/list" => self
                .handle_tools_list()
                .await
                .map(ServerResult::ListToolsResult),
            "tools/call" => self
                .handle_tools_call(params, server_events_tx)
                .await
                .map(ServerResult::CallToolResult),
            method => Err(eyre::eyre!("unexpected method: {:#?}", method)),
        }
    }
}
