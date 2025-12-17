use thiserror::Error;

#[derive(Debug, Error)]
pub enum RclMsgError {
    #[error("Fail to parse member definition: {reason}\ninput: {input}")]
    ParseMemberError { input: String, reason: String },

    #[error("{0} can not have default value")]
    InvalidDefaultError(String),

    #[error("Fail to parse default value: {0}")]
    ParseDefaultValueError(String),

    #[error("Fail to parse constant definition: {reason}\ninput: {input}")]
    ParseConstantError { input: String, reason: String },

    #[error("Fail to parse constant value: {0}")]
    ParseConstantValueError(String),

    #[error("Invalid service specification: {0}")]
    InvalidServiceSpecification(String),

    #[error("Invalid action specification: {0}")]
    InvalidActionSpecification(String),
}
