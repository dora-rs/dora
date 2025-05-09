use dora_node_api::dora_core::config::DataId;
use dora_node_api::{into_vec, DoraNode, Event, IntoArrow, Parameter};
use eyre::{Context, Result};
use rustypot::servo::feetech::sts3215::Sts3215Controller;
use std::collections::BTreeMap;
use std::time::Duration;

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let serialportname: String = std::env::var("PORT").context("Serial port name not provided")?;
    let baudrate: u32 = std::env::var("BAUDRATE")
        .unwrap_or_else(|_| "1000000".to_string())
        .parse()
        .context("Invalid baudrate")?;
    let ids = std::env::var("IDS")
        .unwrap_or_else(|_| "1,2,3,4,5,6".to_string())
        .split(&[',', ' '][..])
        .map(|s| s.parse::<u8>().unwrap())
        .collect::<Vec<u8>>();

    let serial_port = serialport::new(serialportname, baudrate)
        .timeout(Duration::from_millis(1000))
        .open()?;

    let mut c = Sts3215Controller::new()
        .with_protocol_v1()
        .with_serial_port(serial_port);

    if let Ok(torque) = std::env::var("TORQUE") {
        let truthies = vec![true; ids.len()];

        c.write_torque_enable(&ids, &truthies)
            .expect("could not enable torque");

        if let Ok(torque_limit) = torque.parse::<u16>() {
            let limits = vec![torque_limit; ids.len()];
            c.write_torque_limit(&ids, &limits)
                .expect("could not enable torque");
        }
    } else {
        let falsies = vec![false; ids.len()];
        c.write_torque_enable(&ids, &falsies)
            .expect("could not enable torque");
    }

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata: _,
                data,
            } => match id.as_str() {
                "tick" => {
                    if let Ok(joints) = c.read_present_position(&ids) {
                        let mut parameter = BTreeMap::new();
                        parameter.insert(
                            "encoding".to_string(),
                            Parameter::String("jointstate".to_string()),
                        );
                        node.send_output(
                            DataId::from("pose".to_string()),
                            parameter,
                            joints.into_arrow(),
                        )
                        .unwrap();
                    };
                }
                "pose" => {
                    let data: Vec<f64> = into_vec(&data).expect("could not cast values");
                    c.write_goal_position(&ids, &data).unwrap();
                }
                other => eprintln!("Received input `{other}`"),
            },
            _ => {}
        }
    }

    Ok(())
}
