use std::sync::Arc;

use futures::channel::oneshot;
use rmcp::model::{
    CallToolRequest, CallToolResult, EmptyResult, Implementation, InitializeResult,
    ListToolsResult, ProtocolVersion, ServerCapabilities, ServerResult, Tool,
};
use rmcp::model::{ClientRequest, JsonRpcRequest};
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
    pub output: String,
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
                output: tool_config.output.clone(),
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
        request: CallToolRequest,
        request_tx: &mpsc::Sender<ServerEvent>,
    ) -> eyre::Result<CallToolResult> {
        let (tx, rx) = oneshot::channel();

        let tool = self
            .tools
            .iter()
            .find(|t| t.inner.name == request.params.name)
            .ok_or_else(|| eyre::eyre!("Tool not found: {}", request.params.name))?;
        request_tx
            .send(ServerEvent::CallNode {
                output: tool.output.clone(),
                data: serde_json::to_string(&request.params).unwrap(),
                reply: tx,
            })
            .await?;

        let data: String = rx.await?;
        serde_json::from_str(&data)
            .map_err(|e| eyre::eyre!("Failed to parse call tool result: {e}"))
    }

    pub async fn handle_request(
        &self,
        rpc_request: JsonRpcRequest<ClientRequest>,
        server_events_tx: &mpsc::Sender<ServerEvent>,
    ) -> eyre::Result<ServerResult> {
        match rpc_request.request {
            ClientRequest::PingRequest(_request) => {
                self.handle_ping().await.map(ServerResult::EmptyResult)
            }
            ClientRequest::InitializeRequest(_request) => self
                .handle_initialize()
                .await
                .map(ServerResult::InitializeResult),
            ClientRequest::ListToolsRequest(_request) => self
                .handle_tools_list()
                .await
                .map(ServerResult::ListToolsResult),
            ClientRequest::CallToolRequest(request) => self
                .handle_tools_call(request, server_events_tx)
                .await
                .map(ServerResult::CallToolResult),
            method => Err(eyre::eyre!("unexpected method: {:#?}", method)),
        }
    }
}
