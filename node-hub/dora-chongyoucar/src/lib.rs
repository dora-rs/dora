// Chongyou Car Control
// Author：Leon <echo_ai@foxmail.com>

mod command;
mod config;
mod enums;
mod error;
mod json_data;

use std::{env, io::Write, time::Duration};

use dora_node_api::{arrow::array::Float64Array, DoraNode, Event};
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

    let mut com = serial::open(&serial_port).wrap_err(Error::Connect)?;
    com.configure(&COM_SETTINGS).wrap_err(Error::SettingsSet)?;
    com.set_timeout(Duration::from_millis(1000))
        .wrap_err(Error::SetTimeout)?;

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
                let datas = cmd.values();
                if datas.len() == 3 {
                    // x， w 和速度 speed
                    let x = datas[0];
                    let w = datas[0];
                    let speed = if datas[0] < 0.0 { 0.0 } else { datas[0] };
                    tx_key.send((speed * x, speed * w)).ok();
                } else if datas.len() == 6 {
                    // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
                    // [x, y, z, rx, ry, rz]
                    let x = datas[0];
                    let w = datas[5];
                    let speed = 0.2;
                    tx_key.send((speed * x, speed * w)).ok();
                }
            }
        }
    }

    Ok(())
}

#[cfg(feature = "python")]
use pyo3::{
    pyfunction, pymodule,
    types::{PyModule, PyModuleMethods},
    wrap_pyfunction, Bound, PyResult, Python,
};

#[cfg(feature = "python")]
#[pyfunction]
fn py_main(_py: Python) -> PyResult<()> {
    pyo3::prepare_freethreaded_python();

    lib_main().map_err(|err| pyo3::exceptions::PyException::new_err(err.to_string()))
}

#[cfg(feature = "python")]
#[pymodule]
fn dora_chongyoucar(_py: Python, m: Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_main, &m)?)?;
    Ok(())
}
