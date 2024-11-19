// Chongyou Car Control
// Author：Leon（李扬）

mod command;
mod config;
mod enums;
mod error;
mod json_data;

use std::{io::Write, time::Duration};

use dora_node_api::{DoraNode, Event};
use error::Error;
use serial::SerialPort;
use tokio::sync::mpsc;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    dotenv::dotenv().ok();

    // serial port
    let serial_port = std::env::var(config::SERIAL_PORT).unwrap_or("/dev/ttyUSB0".to_string());
    let speed = std::env::var("SPEED")
        .unwrap_or("0.2".to_string())
        .parse::<f64>()
        .unwrap_or(0.2_f64);

    // connect serial
    const COM_SETTINGS: serial::PortSettings = serial::PortSettings {
        baud_rate: serial::Baud115200,
        char_size: serial::Bits8,
        parity: serial::ParityNone,
        stop_bits: serial::Stop1,
        flow_control: serial::FlowNone,
    };

    let mut com = serial::open(&serial_port).map_err(|_| Error::SerialConnect)?;
    com.configure(&COM_SETTINGS)
        .map_err(|_| Error::SerialSettingsSet)?;
    com.set_timeout(Duration::from_millis(1000))
        .map_err(|_| Error::SerialSetTimeout)?;

    // msg channel
    let (tx_key, mut rx_key) = mpsc::channel::<(f64, f64)>(100);

    tokio::spawn(async move {
        while let Some((x, w)) = rx_key.recv().await {
            // println!("{:?}", (x, w));
            let data = command::send_speed_to_x4chassis(x, 0.0, w);
            com.write_all(&data).ok();
        }
    });

    let r = 1.0;

    let (_, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        if let Event::Input {
            id: _,
            metadata: _,
            data,
        } = event
        {
            let received_string: &str = TryFrom::try_from(&data).unwrap();
            match received_string {
                "forward" => {
                    tx_key.send((speed * r, 0.0)).await.ok();
                }
                "left" => {
                    tx_key.send((0.0, speed * r)).await.ok();
                }
                "right" => {
                    tx_key.send((0.0, -speed * r)).await.ok();
                }
                "backward" => {
                    tx_key.send((-speed * r, 0.0)).await.ok();
                }
                "stop" => {
                    tx_key.send((0.0, 0.0)).await.ok();
                }
                _ => {}
            }
        }
    }

    Ok(())
}
