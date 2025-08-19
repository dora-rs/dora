pub(crate) fn gen_call_id() -> String {
    format!("call-{}", uuid::Uuid::new_v4())
}
pub(crate) fn gen_chat_id() -> String {
    format!("chatcmpl-{}", uuid::Uuid::new_v4())
}

pub fn get_env_or_value(value: &str) -> String {
    if let Some(stripped) = value.strip_prefix("env:") {
        std::env::var(stripped).unwrap_or_else(|_| value.to_string())
    } else {
        value.to_string()
    }
}
