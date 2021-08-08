use anyhow::Result;
use rclrust::prelude::*;
use rclrust::qos::QoSProfile;
use rclrust_msg::std_msgs::msg::String as String_;

fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let node = ctx.create_node("examples_subscriber")?;
    let logger = node.logger();

    let _subscription = node.create_subscription(
        "message",
        move |msg: String_| {
            rclrust_info!(logger, "{}", msg.data);
        },
        &QoSProfile::default(),
    )?;

    rclrust::spin(&node)?;

    Ok(())
}
