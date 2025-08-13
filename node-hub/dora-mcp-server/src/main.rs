use std::collections::HashMap;
use std::sync::Arc;

use dora_node_api::{
    DoraNode, Event, MetadataParameters, Parameter,
    arrow::array::{AsArray, StringArray},
    dora_core::config::DataId,
    merged::{MergeExternalSend, MergedEvent},
};

use eyre::{Context, ContextCompat};
use futures::channel::oneshot;
use rmcp::model::{ClientRequest, JsonRpcRequest};
use salvo::cors::*;
use salvo::prelude::*;
use tokio::sync::mpsc;

mod mcp_server;
use mcp_server::McpServer;
mod error;
mod routing;
use error::AppError;
mod config;
mod utils;
use config::Config;
use utils::gen_call_id;

pub type AppResult<T> = Result<T, crate::AppError>;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    config::init();

    let (server_events_tx, server_events_rx) = mpsc::channel(3);
    let server_events = tokio_stream::wrappers::ReceiverStream::new(server_events_rx);

    let mut reply_channels: HashMap<String, oneshot::Sender<String>> = HashMap::new();

    let config = config::get();
    let mcp_server = Arc::new(McpServer::new(config));

    salvo::http::request::set_global_secure_max_size(8_000_000); // set max size to 8MB
    let acceptor = TcpListener::new(&config.listen_addr).bind().await;
    tokio::spawn({
        let server_events_tx = server_events_tx.clone();
        let mcp_server = mcp_server.clone();
        async move {
            let service = Service::new(routing::root(
                config.endpoint.clone(),
                mcp_server,
                server_events_tx.clone(),
            ))
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
                    data,
                    reply,
                } => {
                    let mut metadata = MetadataParameters::default();
                    let call_id = gen_call_id();
                    metadata.insert("__dora_call_id".into(), Parameter::String(call_id.clone()));
                    node.send_output(
                        DataId::from(output.clone()),
                        metadata,
                        StringArray::from(vec![data]),
                    )
                    .context("failed to send dora output")?;

                    reply_channels.insert(call_id, reply);
                }
            },
            MergedEvent::Dora(event) => match event {
                Event::Input { id, data, metadata } => {
                    match id.as_str() {
                        "request" => {
                            let data = data.as_string::<i32>().iter().fold(
                                "".to_string(),
                                |mut acc, s| {
                                    if let Some(s) = s {
                                        acc.push('\n');
                                        acc.push_str(s);
                                    }
                                    acc
                                },
                            );

                            let request =
                                serde_json::from_str::<JsonRpcRequest<ClientRequest>>(&data)
                                    .context("failed to parse call tool from string")?;

                            if let Ok(result) =
                                mcp_server.handle_request(request, &server_events_tx).await
                            {
                                node.send_output(
                                    DataId::from("response".to_owned()),
                                    metadata.parameters,
                                    StringArray::from(vec![
                                        serde_json::to_string(&result).unwrap(),
                                    ]),
                                )
                                .context("failed to send dora output")?;
                            }
                        }
                        _ => {
                            let Some(Parameter::String(call_id)) =
                                metadata.parameters.get("__dora_call_id")
                            else {
                                tracing::warn!("No call ID found in metadata for id: {}", id);
                                continue;
                            };
                            let reply_channel =
                                reply_channels.remove(call_id).context("no reply channel")?;
                            let data = data.as_string::<i32>();
                            let data = data.iter().fold("".to_string(), |mut acc, s| {
                                if let Some(s) = s {
                                    acc.push('\n');
                                    acc.push_str(s);
                                }
                                acc
                            });
                            if reply_channel.send(data).is_err() {
                                tracing::warn!("failed to send reply because channel closed early");
                            }
                            // node.send_output(DataId::from("response".to_owned()), metadata, data)
                            //     .context("failed to send dora output")?;
                        }
                    };
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

enum ServerEvent {
    Result(eyre::Result<()>),
    CallNode {
        output: String,
        data: String,
        reply: oneshot::Sender<String>,
    },
}
