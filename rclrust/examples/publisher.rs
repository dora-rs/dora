use std::thread::sleep;
use std::time::Duration;

use anyhow::Result;
use rclrust::qos::QoSProfile;
use rclrust_msg::std_msgs::msg::String as String_;

fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let node = ctx.create_node("examples_publisher")?;
    let publisher = node.create_publisher::<String_>("message", &QoSProfile::default())?;

    let mut count = 0;
    while count < 100 {
        publisher.publish(&String_ {
            data: format!("hello {}", count),
        })?;
        count += 1;
        sleep(Duration::from_secs_f32(0.1));
    }

    Ok(())
}
