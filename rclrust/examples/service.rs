use anyhow::Result;
use rclrust::qos::QoSProfile;
use rclrust_msg::example_interfaces::srv::{AddTwoInts, AddTwoInts_Response};

fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let node = ctx.create_node("examples_service")?;

    let _service = node.create_service::<AddTwoInts, _>(
        "add_ints",
        move |req| AddTwoInts_Response { sum: req.a + req.b },
        &QoSProfile::default(),
    )?;

    rclrust::spin(&node)?;

    Ok(())
}
