use dora_api::{self, config::DataId, DoraNode};
use std::time::Duration;
use time::OffsetDateTime;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let operator = DoraNode::init_from_env().await?;

    let mut interval = tokio::time::interval(Duration::from_millis(20));

    let time_output = DataId::from("time".to_owned());
    for _ in 0..400 {
        interval.tick().await;
        let now = OffsetDateTime::now_utc().to_string();
        operator.send_output(&time_output, now.as_bytes()).await?;
    }

    Ok(())
}
