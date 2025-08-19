use salvo::async_trait;
use salvo::http::{StatusCode, StatusError};
use salvo::prelude::{Depot, Request, Response, Writer};
use thiserror::Error;

#[derive(Error, Debug)]
pub enum AppError {
    #[error("public: `{0}`")]
    Public(String),
    #[error("internal: `{0}`")]
    Internal(String),
    #[error("salvo internal error: `{0}`")]
    Salvo(#[from] ::salvo::Error),
    #[error("serde json: `{0}`")]
    SerdeJson(#[from] serde_json::error::Error),
    #[error("http: `{0}`")]
    StatusError(#[from] salvo::http::StatusError),
    #[error("http parse: `{0}`")]
    HttpParse(#[from] salvo::http::ParseError),
    #[error("recv: `{0}`")]
    Recv(#[from] tokio::sync::oneshot::error::RecvError),
    #[error("canceled: `{0}`")]
    Canceled(#[from] futures::channel::oneshot::Canceled),
    #[error("error report: `{0}`")]
    ErrReport(#[from] eyre::Report),
    // #[error("reqwest: `{0}`")]
    // Reqwest(#[from] reqwest::Error),
}

impl AppError {
    pub fn public<S: Into<String>>(msg: S) -> Self {
        Self::Public(msg.into())
    }

    pub fn internal<S: Into<String>>(msg: S) -> Self {
        Self::Internal(msg.into())
    }
}

#[async_trait]
impl Writer for AppError {
    async fn write(mut self, _req: &mut Request, _depot: &mut Depot, res: &mut Response) {
        let code = match &self {
            AppError::StatusError(e) => e.code,
            _ => StatusCode::INTERNAL_SERVER_ERROR,
        };
        res.status_code(code);
        let data = match self {
            AppError::Salvo(e) => StatusError::internal_server_error().brief(e.to_string()),
            AppError::Public(msg) => StatusError::internal_server_error().brief(msg),
            AppError::Internal(_msg) => StatusError::internal_server_error(),
            AppError::StatusError(e) => e,
            e => StatusError::internal_server_error().brief(e.to_string()),
        };
        res.render(data);
    }
}
