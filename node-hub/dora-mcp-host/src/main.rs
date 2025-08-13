use std::collections::HashMap;

use dora_node_api::{
    arrow::array::{AsArray, StringArray},
    dora_core::config::DataId,
    merged::{MergeExternalSend, MergedEvent},
    DoraNode, Event, MetadataParameters, Parameter,
};
use eyre::{Context, ContextCompat};
use futures::channel::oneshot;
use outfox_openai::spec::{
    ChatChoice, ChatCompletionResponseMessage, CompletionUsage, CreateChatCompletionRequest,
    CreateChatCompletionResponse, FinishReason, Role,
};
use salvo::cors::*;
use salvo::prelude::*;
use tokio::sync::mpsc;

mod client;
mod error;
mod routing;
mod utils;
use error::AppError;
mod config;
mod session;
use session::ChatSession;
mod tool;
use tool::get_mcp_tools;
use utils::gen_call_id;

pub type AppResult<T> = Result<T, AppError>;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    config::init();

    let (server_events_tx, server_events_rx) = mpsc::channel(3);
    let server_events = tokio_stream::wrappers::ReceiverStream::new(server_events_rx);

    let config = config::get();
    let chat_session = config
        .create_session(server_events_tx.clone())
        .await
        .context("failed to create chat session")?;

    let mut reply_channels: HashMap<
        String,
        (
            oneshot::Sender<CreateChatCompletionResponse>,
            u32,
            Option<String>,
        ),
    > = HashMap::new();

    let acceptor = TcpListener::new(&config.listen_addr).bind().await;
    tokio::spawn(async move {
        let service = Service::new(routing::root(config.endpoint.clone(), chat_session.into()))
            .hoop(
                Cors::new()
                    .allow_origin(AllowOrigin::any())
                    .allow_methods(AllowMethods::any())
                    .allow_headers(AllowHeaders::any())
                    .into_handler(),
            );
        Server::new(acceptor).serve(service).await;
        if let Err(err) = server_events_tx.send(ServerEvent::Result(Ok(()))).await {
            tracing::warn!("server result channel closed: {err}");
        }
    });

    let (mut node, events) = DoraNode::init_from_env()?;

    let merged = events.merge_external_send(server_events);
    let events = futures::executor::block_on_stream(merged);

    for event in events {
        match event {
            MergedEvent::External(event) => match event {
                ServerEvent::Result(server_result) => {
                    server_result.context("server failed")?;
                    break;
                }
                ServerEvent::CallNode {
                    output,
                    request,
                    reply,
                } => {
                    let mut metadata = MetadataParameters::default();
                    let call_id = gen_call_id();
                    metadata.insert("__dora_call_id".into(), Parameter::String(call_id.clone()));
                    let texts = request
                        .messages
                        .iter()
                        .map(|msg| msg.to_texts().join("\n"))
                        .collect::<Vec<_>>();
                    node.send_output(
                        DataId::from(output),
                        Default::default(),
                        StringArray::from(texts),
                    )
                    .context("failed to send dora output")?;

                    reply_channels.insert(call_id, (reply, 0_u32, Some(request.model)));
                }
            },
            MergedEvent::Dora(event) => match event {
                Event::Input { id, data, metadata } => {
                    let Some(Parameter::String(call_id)) =
                        metadata.parameters.get("__dora_call_id")
                    else {
                        tracing::warn!("No call ID found in metadata for id: {}", id);
                        continue;
                    };
                    let (reply_channel, prompt_tokens, model) =
                        reply_channels.remove(call_id).context("no reply channel")?;
                    let data = data.as_string::<i32>();
                    let data = data.iter().fold("".to_string(), |mut acc, s| {
                        if let Some(s) = s {
                            acc.push('\n');
                            acc.push_str(s);
                        }
                        acc
                    });

                    let data = CreateChatCompletionResponse {
                        id: format!("completion-{}", uuid::Uuid::new_v4()),
                        object: "chat.completion".to_string(),
                        created: chrono::Utc::now().timestamp() as u32,
                        model: model.unwrap_or_default(),
                        usage: Some(CompletionUsage {
                            prompt_tokens,
                            completion_tokens: data.len() as u32,
                            total_tokens: prompt_tokens + data.len() as u32,
                            prompt_tokens_details: None,
                            completion_tokens_details: None,
                        }),
                        choices: vec![ChatChoice {
                            index: 0,
                            message: ChatCompletionResponseMessage {
                                role: Role::Assistant,
                                content: Some(data),
                                tool_calls: None,
                                audio: None,
                                refusal: None,
                            },
                            finish_reason: Some(FinishReason::Stop),
                            logprobs: None,
                        }],
                        service_tier: None,
                        system_fingerprint: None,
                    };

                    if reply_channel.send(data).is_err() {
                        tracing::warn!(
                            "failed to send chat completion reply because channel closed early"
                        );
                    }
                }
                Event::Stop(_) => {
                    break;
                }
                Event::InputClosed { id, .. } => {
                    tracing::info!("Input channel closed for id: {}", id);
                }
                event => {
                    eyre::bail!("unexpected event: {:#?}", event)
                }
            },
        }
    }

    Ok(())
}

#[allow(clippy::large_enum_variant)]
pub enum ServerEvent {
    Result(eyre::Result<()>),
    CallNode {
        output: String,
        request: CreateChatCompletionRequest,
        reply: oneshot::Sender<CreateChatCompletionResponse>,
    },
}
