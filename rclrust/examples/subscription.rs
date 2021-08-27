use std::sync::Arc;

use anyhow::Result;
use rclrust::{qos::QoSProfile, rclrust_info};
use rclrust_msg::std_msgs::msg::String as String_;

#[tokio::main]
async fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let mut node = ctx.create_node("examples_subscriber")?;
    let logger = node.logger();

    let _subscription = node.create_subscription(
        "message",
        move |msg: Arc<String_>| {
            rclrust_info!(logger, "{}", msg.data);
        },
        &QoSProfile::default(),
    )?;

    node.wait();
    Ok(())
}
