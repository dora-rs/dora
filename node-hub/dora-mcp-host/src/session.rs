use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use eyre::Result;
use outfox_openai::spec::{
    ChatCompletionRequestMessage, ChatCompletionRequestUserMessage,
    ChatCompletionRequestUserMessageContent, ChatCompletionResponseMessage, ChatCompletionTool,
    ChatCompletionToolType, CreateChatCompletionRequest, CreateChatCompletionResponse,
    FunctionCall, FunctionObject,
};

use crate::client::ChatClient;
use crate::config::ModelConfig;
use crate::tool::ToolSet;

pub struct ChatSession {
    pub chat_clients: HashMap<String, Arc<dyn ChatClient>>,
    pub models: Vec<ModelConfig>,
    pub tool_set: ToolSet,
    pub messages: Mutex<Vec<ChatCompletionRequestMessage>>,
}

impl ChatSession {
    pub fn new(
        chat_clients: HashMap<String, Arc<dyn ChatClient>>,
        models: Vec<ModelConfig>,
        prompts: Vec<ChatCompletionRequestMessage>,
        tool_set: ToolSet,
    ) -> Self {
        Self {
            chat_clients,
            models,
            tool_set,
            messages: Mutex::new(prompts),
        }
    }

    // pub fn default_model(&self) -> Option<&str> {
    //     let model = self
    //         .models
    //         .iter()
    //         .find(|model| model.default)
    //         .map(|model| &*model.id);
    //     if model.is_none() {
    //         self.models.first().map(|model| &*model.id)
    //     } else {
    //         model
    //     }
    // }

    pub fn route(&self, model: &str) -> Option<(Arc<dyn ChatClient>, String)> {
        let model = self.models.iter().find(|m| m.id == model)?;
        let route = &model.route;
        let client = self.chat_clients.get(&route.provider)?;
        Some((
            client.clone(),
            route.model.clone().unwrap_or(model.id.clone()),
        ))
    }

    // pub fn add_prompt(&mut self, prompt: impl Into<ChatCompletionRequestMessage>) {
    //     let mut messages = self.messages.lock().expect("messages should locked");
    //     messages.push(prompt.into());
    // }

    // pub fn get_tools(&self) -> Vec<Arc<dyn ToolTrait>> {
    //     self.tool_set.tools()
    // }

    pub async fn analyze_tool_call(&self, response: &ChatCompletionResponseMessage) {
        let mut tool_calls_func = Vec::new();
        if let Some(tool_calls) = response.tool_calls.as_ref() {
            for tool_call in tool_calls {
                // if tool_call.r#type == "function" {
                tool_calls_func.push(tool_call.function.clone());
                // }
            }
        } else {
            // check if message contains tool call
            if let Some(text) = &response.content {
                if text.contains("Tool:") {
                    let lines: Vec<&str> = text.split('\n').collect();
                    // simple parse tool call
                    let mut tool_name = None;
                    let mut args_text = Vec::new();
                    let mut parsing_args = false;

                    for line in lines {
                        if line.starts_with("Tool:") {
                            tool_name = line.strip_prefix("Tool:").map(|s| s.trim().to_string());
                            parsing_args = false;
                        } else if line.starts_with("Inputs:") {
                            parsing_args = true;
                        } else if parsing_args {
                            args_text.push(line.trim());
                        }
                    }
                    if let Some(name) = tool_name {
                        tool_calls_func.push(FunctionCall {
                            name,
                            arguments: args_text.join("\n"),
                        });
                    }
                }
            }
        }
        // call tool
        for tool_call in tool_calls_func {
            let tool = self.tool_set.get_tool(&tool_call.name);
            if let Some(tool) = tool {
                // call tool
                let args = serde_json::from_str::<serde_json::Value>(&tool_call.arguments)
                    .unwrap_or_default();
                match tool.call(args).await {
                    Ok(result) => {
                        if result.is_error.is_some_and(|b| b) {
                            let mut messages =
                                self.messages.lock().expect("messages should locked");
                            messages.push(
                                ChatCompletionRequestUserMessage::new(
                                    "tool call failed, mcp call error",
                                )
                                .into(),
                            );
                        } else if let Some(contents) = &result.content {
                            contents.iter().for_each(|content| {
                                if let Some(content_text) = content.as_text() {
                                    let json_result = serde_json::from_str::<serde_json::Value>(
                                        &content_text.text,
                                    )
                                    .unwrap_or_default();
                                    let pretty_result =
                                        serde_json::to_string_pretty(&json_result).unwrap();
                                    tracing::debug!("call tool result: {}", pretty_result);
                                    let mut messages =
                                        self.messages.lock().expect("messages should locked");
                                    messages.push(
                                        ChatCompletionRequestUserMessage::new(format!(
                                            "call tool result: {pretty_result}"
                                        ))
                                        .into(),
                                    );
                                }
                            });
                        }
                    }
                    Err(e) => {
                        tracing::error!("tool call failed: {}", e);
                        let mut messages = self.messages.lock().expect("messages should locked");
                        messages.push(
                            ChatCompletionRequestUserMessage {
                                content: ChatCompletionRequestUserMessageContent::Text(format!(
                                    "tool call failed: {e}"
                                )),
                                name: None,
                            }
                            .into(),
                        );
                    }
                }
            } else {
                println!("tool not found: {}", tool_call.name);
            }
        }
    }
    pub async fn chat(
        &self,
        mut request: CreateChatCompletionRequest,
    ) -> Result<CreateChatCompletionResponse> {
        {
            let mut messages = self.messages.lock().expect("messages should locked");
            for message in std::mem::take(&mut request.messages) {
                messages.push(message);
            }
            request.messages = messages.clone();
        }
        let tools = self.tool_set.tools();
        let tool_definitions = if !tools.is_empty() {
            Some(
                tools
                    .iter()
                    .map(|tool| ChatCompletionTool {
                        kind: ChatCompletionToolType::Function,
                        function: FunctionObject {
                            name: tool.name(),
                            description: Some(tool.description()),
                            parameters: Some(tool.parameters()),
                            strict: None,
                        },
                    })
                    .collect(),
            )
        } else {
            None
        };

        let (client, model) = self.route(&request.model).expect("failed to route model");
        request.model = model.clone();
        request.tools = tool_definitions;

        // send request
        let response = client.complete(request).await?;
        // get choice
        if let Some(choice) = response.choices.first() {
            // analyze tool call
            self.analyze_tool_call(&choice.message).await;
            let request = {
                let messages = self.messages.lock().expect("messages should locked");
                CreateChatCompletionRequest::new(model, messages.clone())
            };
            client.complete(request).await
        } else {
            Ok(response)
        }
    }
}
