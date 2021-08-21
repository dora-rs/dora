use anyhow::Result;
use rclrust::{
    rclrust_debug,
    rclrust_error,
    rclrust_fatal,
    rclrust_info,
    rclrust_warn,
    Clock,
    Logger,
};
use rclrust_msg::geometry_msgs::msg::Twist;

fn main() -> Result<()> {
    let _ctx = rclrust::init()?;

    let logger = Logger::new("log example");

    rclrust_debug!(logger, "debug log: {}", 20);
    rclrust_info!(logger, "info log: {:?}", Twist::default());
    rclrust_warn!(logger, "warn log: {} {} {}", 20.3, 20, true);
    rclrust_error!(
        logger,
        "error log: {:?}",
        Clock::ros().unwrap().now().unwrap()
    );
    rclrust_fatal!(logger, "fatal log: plain message");

    Ok(())
}
