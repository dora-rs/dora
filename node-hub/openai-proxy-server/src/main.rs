use dora_node_api::{self, dora_core::config::DataId, merged::MergeExternalSend, DoraNode, Event};

use eyre::{Context, ContextCompat};
use futures::channel::oneshot::{self, Canceled};
use hyper::{
    body::{to_bytes, Body, HttpBody},
    header,
    server::conn::AddrStream,
    service::{make_service_fn, service_fn},
    Request, Response, Server, StatusCode,
};
use message::{
    ChatCompletionObject, ChatCompletionObjectChoice, ChatCompletionObjectMessage,
    ChatCompletionRequest, ChatCompletionRequestMessage, Usage,
};
use std::{
    collections::VecDeque,
    net::SocketAddr,
    path::{Path, PathBuf},
};
use tokio::{net::TcpListener, sync::mpsc};
use tracing::{error, info};

mod error;
pub mod message;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let web_ui = Path::new("chatbot-ui");
    let port = 8000;
    let addr = SocketAddr::from(([0, 0, 0, 0], port));

    let (server_events_tx, server_events_rx) = mpsc::channel(3);
    let server_events = tokio_stream::wrappers::ReceiverStream::new(server_events_rx);

    let server_result_tx = server_events_tx.clone();
    let new_service = make_service_fn(move |conn: &AddrStream| {
        // log socket address
        info!(target: "stdout", "remote_addr: {}, local_addr: {}", conn.remote_addr().to_string(), conn.local_addr().to_string());

        // web ui
        let web_ui = web_ui.to_string_lossy().to_string();
        let server_events_tx = server_events_tx.clone();
        async move {
            let service = service_fn(move |req| {
                handle_request(req, web_ui.clone(), server_events_tx.clone())
            });
            Ok::<_, eyre::Error>(service)
        }
    });

    let tcp_listener = TcpListener::bind(addr).await.unwrap();
    info!(target: "stdout", "Listening on {}", addr);

    let server = Server::from_tcp(tcp_listener.into_std().unwrap())
        .unwrap()
        .serve(new_service);

    tokio::spawn(async move {
        let result = server.await.context("server task failed");
        if let Err(err) = server_result_tx.send(ServerEvent::Result(result)).await {
            tracing::warn!("server result channel closed: {err}");
        }
    });

    let (mut node, events) = DoraNode::init_from_env()?;

    let merged = events.merge_external_send(server_events);
    let events = futures::executor::block_on_stream(merged);

    let output_id = DataId::from("chat_completion_request".to_owned());
    let mut reply_channels = VecDeque::new();

    for event in events {
        match event {
            dora_node_api::merged::MergedEvent::External(event) => match event {
                ServerEvent::Result(server_result) => {
                    server_result.context("server failed")?;
                    break;
                }
                ServerEvent::ChatCompletionRequest { request, reply } => {
                    let message = request
                        .messages
                        .into_iter()
                        .find_map(|m| match m {
                            ChatCompletionRequestMessage::User(message) => Some(message),
                            _ => None,
                        })
                        .context("no user message found");
                    match message {
                        Ok(message) => match message.content() {
                            message::ChatCompletionUserMessageContent::Text(content) => {
                                node.send_output_bytes(
                                    output_id.clone(),
                                    Default::default(),
                                    content.len(),
                                    content.as_bytes(),
                                )
                                .context("failed to send dora output")?;
                                reply_channels.push_back((
                                    reply,
                                    content.as_bytes().len() as u64,
                                    request.model,
                                ));
                            }
                            message::ChatCompletionUserMessageContent::Parts(_) => {
                                if reply
                                    .send(Err(eyre::eyre!("unsupported message content")))
                                    .is_err()
                                {
                                    tracing::warn!("failed to send chat completion reply because channel closed early");
                                };
                            }
                        },
                        Err(err) => {
                            if reply.send(Err(err)).is_err() {
                                tracing::warn!("failed to send chat completion reply error because channel closed early");
                            }
                        }
                    }
                }
            },
            dora_node_api::merged::MergedEvent::Dora(event) => match event {
                Event::Input {
                    id,
                    data,
                    metadata: _,
                } => {
                    match id.as_str() {
                        "completion_reply" => {
                            let (reply_channel, prompt_tokens, model) =
                                reply_channels.pop_front().context("no reply channel")?;
                            let data = TryFrom::try_from(&data)
                                .with_context(|| format!("invalid reply data: {data:?}"))
                                .map(|s: &[u8]| ChatCompletionObject {
                                    id: format!("completion-{}", uuid::Uuid::new_v4()),
                                    object: "chat.completion".to_string(),
                                    created: chrono::Utc::now().timestamp() as u64,
                                    model: model.unwrap_or_default(),
                                    choices: vec![ChatCompletionObjectChoice {
                                        index: 0,
                                        message: ChatCompletionObjectMessage {
                                            role: message::ChatCompletionRole::Assistant,
                                            content: Some(String::from_utf8_lossy(s).to_string()),
                                            tool_calls: Vec::new(),
                                            function_call: None,
                                        },
                                        finish_reason: message::FinishReason::stop,
                                        logprobs: None,
                                    }],
                                    usage: Usage {
                                        prompt_tokens,
                                        completion_tokens: s.len() as u64,
                                        total_tokens: prompt_tokens + s.len() as u64,
                                    },
                                });

                            if reply_channel.send(data).is_err() {
                                tracing::warn!("failed to send chat completion reply because channel closed early");
                            }
                        }
                        _ => eyre::bail!("unexpected input id: {}", id),
                    };
                }
                Event::Stop => {
                    break;
                }
                event => {
                    println!("Event: {event:#?}")
                }
            },
        }
    }

    Ok(())
}

enum ServerEvent {
    Result(eyre::Result<()>),
    ChatCompletionRequest {
        request: ChatCompletionRequest,
        reply: oneshot::Sender<eyre::Result<ChatCompletionObject>>,
    },
}

// Forked from https://github.com/LlamaEdge/LlamaEdge/blob/6bfe9c12c85bf390c47d6065686caeca700feffa/llama-api-server/src/main.rs
async fn handle_request(
    req: Request<Body>,
    web_ui: String,
    request_tx: mpsc::Sender<ServerEvent>,
) -> Result<Response<Body>, hyper::Error> {
    let path_str = req.uri().path();
    let path_buf = PathBuf::from(path_str);
    let mut path_iter = path_buf.iter();
    path_iter.next(); // Must be Some(OsStr::new(&path::MAIN_SEPARATOR.to_string()))
    let root_path = path_iter.next().unwrap_or_default();
    let root_path = "/".to_owned() + root_path.to_str().unwrap_or_default();

    // log request
    {
        let method = hyper::http::Method::as_str(req.method()).to_string();
        let path = req.uri().path().to_string();
        let version = format!("{:?}", req.version());
        if req.method() == hyper::http::Method::POST {
            let size: u64 = match req.headers().get("content-length") {
                Some(content_length) => content_length.to_str().unwrap().parse().unwrap(),
                None => 0,
            };

            info!(target: "stdout", "method: {}, http_version: {}, content-length: {}", method, version, size);
            info!(target: "stdout", "endpoint: {}", path);
        } else {
            info!(target: "stdout", "method: {}, http_version: {}", method, version);
            info!(target: "stdout", "endpoint: {}", path);
        }
    }

    let response = match root_path.as_str() {
        "/echo" => Response::new(Body::from("echo test")),
        "/v1" => handle_llama_request(req, request_tx).await,
        _ => static_response(path_str, web_ui),
    };

    // log response
    {
        let status_code = response.status();
        if status_code.as_u16() < 400 {
            // log response
            let response_version = format!("{:?}", response.version());
            info!(target: "stdout", "response_version: {}", response_version);
            let response_body_size: u64 = response.body().size_hint().lower();
            info!(target: "stdout", "response_body_size: {}", response_body_size);
            let response_status = status_code.as_u16();
            info!(target: "stdout", "response_status: {}", response_status);
            let response_is_success = status_code.is_success();
            info!(target: "stdout", "response_is_success: {}", response_is_success);
        } else {
            let response_version = format!("{:?}", response.version());
            error!(target: "stdout", "response_version: {}", response_version);
            let response_body_size: u64 = response.body().size_hint().lower();
            error!(target: "stdout", "response_body_size: {}", response_body_size);
            let response_status = status_code.as_u16();
            error!(target: "stdout", "response_status: {}", response_status);
            let response_is_success = status_code.is_success();
            error!(target: "stdout", "response_is_success: {}", response_is_success);
            let response_is_client_error = status_code.is_client_error();
            error!(target: "stdout", "response_is_client_error: {}", response_is_client_error);
            let response_is_server_error = status_code.is_server_error();
            error!(target: "stdout", "response_is_server_error: {}", response_is_server_error);
        }
    }

    Ok(response)
}

fn static_response(path_str: &str, root: String) -> Response<Body> {
    let path = match path_str {
        "/" => "/index.html",
        _ => path_str,
    };

    let mime = mime_guess::from_path(path);

    match std::fs::read(format!("{root}/{path}")) {
        Ok(content) => Response::builder()
            .status(StatusCode::OK)
            .header(header::CONTENT_TYPE, mime.first_or_text_plain().to_string())
            .body(Body::from(content))
            .unwrap(),
        Err(_) => {
            let body = Body::from(std::fs::read(format!("{root}/404.html")).unwrap_or_default());
            Response::builder()
                .status(StatusCode::NOT_FOUND)
                .header(header::CONTENT_TYPE, "text/html")
                .body(body)
                .unwrap()
        }
    }
}

// Forked from https://github.com/LlamaEdge/LlamaEdge/blob/6bfe9c12c85bf390c47d6065686caeca700feffa/llama-api-server/src/backend/mod.rs#L8
async fn handle_llama_request(
    req: Request<Body>,
    request_tx: mpsc::Sender<ServerEvent>,
) -> Response<Body> {
    match req.uri().path() {
        "/v1/chat/completions" => chat_completions_handler(req, request_tx).await,
        // "/v1/completions" => ggml::completions_handler(req).await,
        // "/v1/models" => ggml::models_handler().await,
        // "/v1/embeddings" => ggml::embeddings_handler(req).await,
        // "/v1/files" => ggml::files_handler(req).await,
        // "/v1/chunks" => ggml::chunks_handler(req).await,
        // "/v1/info" => ggml::server_info_handler().await,
        // path if path.starts_with("/v1/files/") => ggml::files_handler(req).await,
        path => error::invalid_endpoint(path),
    }
}

// Forked from https://github.com/LlamaEdge/LlamaEdge/blob/6bfe9c12c85bf390c47d6065686caeca700feffa/llama-api-server/src/backend/ggml.rs#L301
async fn chat_completions_handler(
    mut req: Request<Body>,
    request_tx: mpsc::Sender<ServerEvent>,
) -> Response<Body> {
    info!(target: "stdout", "Handling the coming chat completion request.");

    if req.method().eq(&hyper::http::Method::OPTIONS) {
        let result = Response::builder()
            .header("Access-Control-Allow-Origin", "*")
            .header("Access-Control-Allow-Methods", "*")
            .header("Access-Control-Allow-Headers", "*")
            .header("Content-Type", "application/json")
            .body(Body::empty());

        match result {
            Ok(response) => return response,
            Err(e) => {
                let err_msg = e.to_string();

                // log
                error!(target: "stdout", "{}", &err_msg);

                return error::internal_server_error(err_msg);
            }
        }
    }

    info!(target: "stdout", "Prepare the chat completion request.");

    // parse request
    let body_bytes = match to_bytes(req.body_mut()).await {
        Ok(body_bytes) => body_bytes,
        Err(e) => {
            let err_msg = format!("Fail to read buffer from request body. {}", e);

            // log
            error!(target: "stdout", "{}", &err_msg);

            return error::internal_server_error(err_msg);
        }
    };
    let mut chat_request: ChatCompletionRequest = match serde_json::from_slice(&body_bytes) {
        Ok(chat_request) => chat_request,
        Err(e) => {
            let mut err_msg = format!("Fail to deserialize chat completion request: {}.", e);

            if let Ok(json_value) = serde_json::from_slice::<serde_json::Value>(&body_bytes) {
                err_msg = format!("{}\njson_value: {}", err_msg, json_value);
            }

            // log
            error!(target: "stdout", "{}", &err_msg);

            return error::bad_request(err_msg);
        }
    };

    // check if the user id is provided
    if chat_request.user.is_none() {
        chat_request.user = Some(gen_chat_id())
    };
    let id = chat_request.user.clone().unwrap();

    // log user id
    info!(target: "stdout", "user: {}", chat_request.user.clone().unwrap());

    let (tx, rx) = oneshot::channel();
    if let Err(err) = request_tx
        .send(ServerEvent::ChatCompletionRequest {
            request: chat_request,
            reply: tx,
        })
        .await
        .context("failed to send request")
    {
        return error::internal_server_error(format!("{err:?}"));
    }

    let res = match rx
        .await
        .unwrap_or_else(|Canceled| Err(eyre::eyre!("result channel closed early")))
    {
        Ok(chat_completion_object) => {
            // serialize chat completion object
            let s = match serde_json::to_string(&chat_completion_object) {
                Ok(s) => s,
                Err(e) => {
                    let err_msg = format!("Failed to serialize chat completion object. {}", e);

                    // log
                    error!(target: "stdout", "{}", &err_msg);

                    return error::internal_server_error(err_msg);
                }
            };

            // return response
            let result = Response::builder()
                .header("Access-Control-Allow-Origin", "*")
                .header("Access-Control-Allow-Methods", "*")
                .header("Access-Control-Allow-Headers", "*")
                .header("Content-Type", "application/json")
                .header("user", id)
                .body(Body::from(s));

            match result {
                Ok(response) => {
                    // log
                    info!(target: "stdout", "Finish chat completions in non-stream mode");

                    response
                }
                Err(e) => {
                    let err_msg =
                        format!("Failed chat completions in non-stream mode. Reason: {}", e);

                    // log
                    error!(target: "stdout", "{}", &err_msg);

                    error::internal_server_error(err_msg)
                }
            }
        }
        Err(e) => {
            let err_msg = format!("Failed to get chat completions. Reason: {}", e);

            // log
            error!(target: "stdout", "{}", &err_msg);

            error::internal_server_error(err_msg)
        }
    };

    // log
    info!(target: "stdout", "Send the chat completion response.");

    res
}

pub(crate) fn gen_chat_id() -> String {
    format!("chatcmpl-{}", uuid::Uuid::new_v4())
}
