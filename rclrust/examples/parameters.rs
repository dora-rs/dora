use anyhow::Result;
use rclrust::rclrust_info;
use rclrust::{Parameter, ParameterValue};

fn main() -> Result<()> {
    let ctx = rclrust::init()?;
    let node = ctx.create_node("example_parameters")?;
    let logger = node.logger();

    node.declare_parameter("param0", &ParameterValue::integer(20))?;

    rclrust_info!(logger, "param0 = {}", node.get_parameter("param0").unwrap());

    node.set_parameter(Parameter::double("param0", 2.3))?;

    rclrust_info!(logger, "param0 = {}", node.get_parameter("param0").unwrap());

    std::thread::sleep(std::time::Duration::from_secs(20));

    Ok(())
}
