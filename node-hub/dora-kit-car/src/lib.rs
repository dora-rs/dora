// Chongyou Car Control
// Author：Leon <echo_ai@foxmail.com>

mod command;
mod config;
mod enums;
mod error;
mod json_data;

use std::{env, io::Write, time::Duration};

use dora_node_api::{DoraNode, Event, arrow::array::Float64Array};
use error::Error;
use eyre::Context;
use serial::SerialPort;
use std::sync::mpsc;

pub fn lib_main() -> eyre::Result<()> {
    dotenv::dotenv().ok();

    // serial port
    let serial_port = env::var(config::SERIAL_PORT).unwrap_or("/dev/ttyUSB0".to_string());

    // connect serial
    const COM_SETTINGS: serial::PortSettings = serial::PortSettings {
        baud_rate: serial::Baud115200,
        char_size: serial::Bits8,
        parity: serial::ParityNone,
        stop_bits: serial::Stop1,
        flow_control: serial::FlowNone,
    };

    let mut com = serial::open(&serial_port).wrap_err(Error::Connect(serial_port))?;
    com.configure(&COM_SETTINGS)
        .wrap_err(Error::SettingsSet(format!("{COM_SETTINGS:?}")))?;
    com.set_timeout(Duration::from_millis(1000))
        .wrap_err(Error::SetTimeout("1000ms".to_string()))?;

    // msg channel
    let (tx_key, rx_key) = mpsc::channel::<(f64, f64)>();

    std::thread::spawn(move || {
        while let Ok((x, w)) = rx_key.recv() {
            let data = command::send_speed_to_x4chassis(x, 0.0, w);
            com.write_all(&data).ok();
        }
    });

    let (_, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        if let Event::Input {
            id: _,
            metadata: _,
            data,
        } = event
        {
            if let Some(cmd) = data.as_any().downcast_ref::<Float64Array>() {
                let data = cmd.values();
                if data.len() == 6 {
                    // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
                    // [x, y, z, rx, ry, rz]
                    let x = data[0];
                    let w = data[5];
                    tx_key.send((x, w)).ok();
                }
            }
        }
    }

    Ok(())
}

#[cfg(feature = "python")]
use pyo3::{
    Bound, PyResult, Python, pyfunction, pymodule,
    types::{PyModule, PyModuleMethods},
    wrap_pyfunction,
};

#[cfg(feature = "python")]
#[pyfunction]
fn py_main(_py: Python) -> eyre::Result<()> {
    lib_main()
}

#[cfg(feature = "python")]
#[pymodule]
fn dora_kit_car(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
