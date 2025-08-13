use std::sync::Arc;

use eyre::Context;
use rmcp::model::{ClientJsonRpcMessage, JsonRpcResponse, JsonRpcVersion2_0};
use salvo::prelude::*;
use tokio::sync::mpsc;

use crate::{AppResult, McpServer, ServerEvent};

pub fn root(
    endpoint: Option<String>,
    mcp_server: Arc<McpServer>,
    server_events_tx: mpsc::Sender<ServerEvent>,
) -> Router {
    Router::with_hoop(affix_state::inject(mcp_server).inject(server_events_tx)).push(
        if let Some(endpoint) = endpoint {
            Router::with_path(endpoint)
        } else {
            Router::new()
        }
        .post(handle_post)
        .delete(handle_delete),
    )
}

#[handler]
async fn handle_delete(res: &mut Response) {
    res.render(Text::Plain("DELETE method is not supported"));
}

#[handler]
async fn handle_post(req: &mut Request, depot: &mut Depot, res: &mut Response) -> AppResult<()> {
    tracing::debug!("Handling the coming chat completion request.");
    let server_events_tx = depot
        .obtain::<mpsc::Sender<ServerEvent>>()
        .expect("server_events_tx must be exists");
    let mcp_server = depot
        .obtain::<Arc<McpServer>>()
        .expect("mcp server must be exists");

    tracing::debug!("Prepare the chat completion request.");

    let rpc_message = serde_json::from_slice::<ClientJsonRpcMessage>(req.payload().await?)
        .context("failed to parse request bodyxxx")?;
    match rpc_message {
        ClientJsonRpcMessage::Request(rpc_request) => {
            let response = JsonRpcResponse {
                jsonrpc: JsonRpcVersion2_0,
                id: rpc_request.id.clone(),
                result: mcp_server
                    .handle_request(rpc_request, server_events_tx)
                    .await
                    .unwrap(),
            };
            res.render(Json(response));
        }
        ClientJsonRpcMessage::Notification(_)
        | ClientJsonRpcMessage::Response(_)
        | ClientJsonRpcMessage::Error(_) => {
            res.render(StatusCode::ACCEPTED);
        }
        _ => {
            res.render(
                StatusError::not_implemented().brief("Batch requests are not supported yet"),
            );
        }
    }

    tracing::debug!("Send the chat completion response.");
    Ok(())
}
