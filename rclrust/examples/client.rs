use std::thread;

use anyhow::Result;
use rclrust::prelude::*;
use rclrust::qos::QoSProfile;
use rclrust_msg::example_interfaces::srv::{AddTwoInts, AddTwoInts_Request};

fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let node = ctx.create_node("examples_client")?;
    let logger = node.logger();

    let client = node.create_client::<AddTwoInts>("add_ints", &QoSProfile::default())?;

    while !client.service_is_available()? {
        std::thread::sleep(std::time::Duration::from_secs_f32(0.1));
    }

    let req = AddTwoInts_Request { a: 17, b: 25 };
    let mut recv = client.send_request(&req)?;

    thread::spawn(move || loop {
        match recv.try_recv() {
            Ok(Some(res)) => {
                rclrust_info!(logger, "{} + {} = {}", req.a, req.b, res.sum);
                return;
            }
            Ok(None) => continue,
            Err(_) => rclrust_error!(logger, "Request is cancelled"),
        }
    });

    rclrust::spin(&node)?;

    Ok(())
}
