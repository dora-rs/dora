use eyre::eyre;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register { machine_id: String },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum RegisterResult {
    Ok,
    Err(String),
}

impl RegisterResult {
    pub fn to_result(self) -> eyre::Result<()> {
        match self {
            RegisterResult::Ok => Ok(()),
            RegisterResult::Err(err) => Err(eyre!(err)),
        }
    }
}
