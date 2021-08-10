use std::thread::sleep;
use std::time::Duration;

use anyhow::Result;
use rclrust::rclrust_info;
use rclrust::qos::QoSProfile;
use rclrust_msg::std_msgs::msg::String as String_;

fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let node = ctx.create_node("examples_publisher")?;
    let logger = node.logger();
    let publisher = node.create_publisher::<String_>("message", &QoSProfile::default())?;

    for count in 0..1000 {
        publisher.publish(&String_ {
            data: format!("hello {}", count),
        })?;
        rclrust_info!(logger, "hello {}", count);
        sleep(Duration::from_secs_f32(0.1));
    }

    Ok(())
}
