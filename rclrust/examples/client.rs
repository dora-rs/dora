use anyhow::Result;
use rclrust::{qos::QoSProfile, rclrust_info};
use rclrust_msg::example_interfaces::srv::{AddTwoInts, AddTwoInts_Request};

#[tokio::main]
async fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let mut node = ctx.create_node("examples_client")?;
    let logger = node.logger();

    let mut client = node.create_client::<AddTwoInts>("add_ints", &QoSProfile::default())?;
    client.wait_service()?;

    let req = AddTwoInts_Request { a: 17, b: 25 };
    let task = client.send_request(&req).await?;
    rclrust_info!(logger, "{} + {} = {}", req.a, req.b, task.sum);

    Ok(())
}
