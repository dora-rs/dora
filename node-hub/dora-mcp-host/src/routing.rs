use std::sync::Arc;

use outfox_openai::spec::{CreateChatCompletionRequest, Model};
use salvo::prelude::*;

use crate::AppResult;
use crate::session::ChatSession;

pub fn root(endpoint: Option<String>, chat_session: Arc<ChatSession>) -> Router {
    Router::with_hoop(affix_state::inject(chat_session))
        .push(
            if let Some(endpoint) = endpoint {
                Router::with_path(endpoint)
            } else {
                Router::new()
            }
            .push(Router::with_path("chat/completions").post(chat_completions))
            .push(Router::with_path("models").get(list_models))
            .push(Router::with_path("embeddings").get(todo))
            .push(Router::with_path("files").get(todo))
            .push(Router::with_path("chunks").get(todo))
            .push(Router::with_path("info").get(todo))
            .push(Router::with_path("realtime").get(todo)),
        )
        .push(Router::with_path("{**path}").get(index))
}

#[handler]
async fn todo(res: &mut Response) {
    res.render(Text::Plain("TODO"));
}
#[handler]
async fn index(res: &mut Response) {
    res.render(Text::Plain("Hello"));
}

#[handler]
async fn list_models(depot: &mut Depot, res: &mut Response) {
    let chat_session = depot
        .obtain::<Arc<ChatSession>>()
        .expect("chat session must be exists");

    let mut models = Vec::new();
    for model in &chat_session.models {
        // TODO: fill correct data
        models.push(Model {
            id: model.id.clone(),
            object: model.object.clone().unwrap_or("object".into()),
            created: model.created.unwrap_or_default(),
            owned_by: model.owned_by.clone().unwrap_or("dora".into()),
        });
    }
    res.render(Json(models));
}

#[handler]
async fn chat_completions(
    req: &mut Request,
    depot: &mut Depot,
    res: &mut Response,
) -> AppResult<()> {
    tracing::info!("Handling the coming chat completion request.");
    let chat_session = depot
        .obtain::<Arc<ChatSession>>()
        .expect("chat session must be exists");

    tracing::info!("Prepare the chat completion request.");

    let mut chat_request = match req.parse_json::<CreateChatCompletionRequest>().await {
        Ok(chat_request) => chat_request,
        Err(e) => {
            println!(
                "parse request error: {e}, payload: {}",
                String::from_utf8_lossy(req.payload().await?)
            );
            return Err(e.into());
        }
    };

    // check if the user id is provided
    if chat_request.user.is_none() {
        chat_request.user = Some(crate::utils::gen_chat_id())
    };
    let id = chat_request.user.clone().unwrap();

    // log user id
    tracing::info!("user: {}", chat_request.user.clone().unwrap());
    // let stream = chat_request.stream;

    // let (tx, rx) = oneshot::channel();
    // request_tx
    //     .send(ServerEvent::CompletionRequest {
    //         request: chat_request,
    //         reply: tx,
    //     })
    //     .await?;

    // if let Some(true) = stream {
    //     // let result = async {
    //     //     let chat_completion_object = rx.await?;
    //     //     Ok::<_, AppError>(serde_json::to_string(&chat_completion_object)?)
    //     // };
    //     let result = chat_session.chat(chat_request).await?;
    //     let stream = futures::stream::once(result);

    //     let _ = res.add_header("Content-Type", "text/event-stream", true);
    //     let _ = res.add_header("Cache-Control", "no-cache", true);
    //     let _ = res.add_header("Connection", "keep-alive", true);
    //     let _ = res.add_header("user", id, true);
    //     res.stream(stream);
    // } else {
    let response = chat_session.chat(chat_request).await?;
    let _ = res.add_header("user", id, true);
    res.render(Json(response));
    // };
    tracing::info!("Send the chat completion response.");
    Ok(())
}
