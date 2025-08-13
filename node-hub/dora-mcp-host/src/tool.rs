use std::{collections::HashMap, sync::Arc};

use eyre::Result;
use rmcp::{
    RoleClient,
    model::{CallToolRequestParam, CallToolResult, Tool as McpTool},
    service::{RunningService, ServerSink},
};
use salvo::async_trait;
use serde_json::Value;

#[async_trait]
pub trait Tool: Send + Sync {
    fn name(&self) -> String;
    fn description(&self) -> String;
    fn parameters(&self) -> Value;
    async fn call(&self, args: Value) -> Result<CallToolResult>;
}

pub struct McpToolAdapter {
    tool: McpTool,
    server: ServerSink,
}

impl McpToolAdapter {
    pub fn new(tool: McpTool, server: ServerSink) -> Self {
        Self { tool, server }
    }
}

#[async_trait]
impl Tool for McpToolAdapter {
    fn name(&self) -> String {
        self.tool.name.clone().to_string()
    }

    fn description(&self) -> String {
        self.tool
            .description
            .clone()
            .unwrap_or_default()
            .to_string()
    }

    fn parameters(&self) -> Value {
        serde_json::to_value(&self.tool.input_schema).unwrap_or(serde_json::json!({}))
    }

    async fn call(&self, args: Value) -> Result<CallToolResult> {
        let arguments = match args {
            Value::Object(map) => Some(map),
            _ => None,
        };
        let call_result = self
            .server
            .call_tool(CallToolRequestParam {
                name: self.tool.name.clone(),
                arguments,
            })
            .await?;

        Ok(call_result)
    }
}
#[derive(Default)]
pub struct ToolSet {
    tools: HashMap<String, Arc<dyn Tool>>,
    clients: HashMap<String, RunningService<RoleClient, ()>>,
}

impl ToolSet {
    pub fn set_clients(&mut self, clients: HashMap<String, RunningService<RoleClient, ()>>) {
        self.clients = clients;
    }

    pub fn add_tool<T: Tool + 'static>(&mut self, tool: T) {
        self.tools.insert(tool.name(), Arc::new(tool));
    }

    pub fn get_tool(&self, name: &str) -> Option<Arc<dyn Tool>> {
        self.tools.get(name).cloned()
    }

    pub fn tools(&self) -> Vec<Arc<dyn Tool>> {
        self.tools.values().cloned().collect()
    }
}

pub async fn get_mcp_tools(server: ServerSink) -> Result<Vec<McpToolAdapter>> {
    let tools = server.list_all_tools().await?;
    Ok(tools
        .into_iter()
        .map(|tool| McpToolAdapter::new(tool, server.clone()))
        .collect())
}
