use crate::config::NodeId;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum Request {
    Register { node_id: NodeId },
    PrepareOutputMessage { len: usize },
    SendOutMessage { id: MessageId },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum Reply {
    RegisterResult(Result<(), String>),
}

type MessageId = String;
