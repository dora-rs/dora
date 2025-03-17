use rustypot::{device::xm, DynamixelSerialIO};
use std::time::{Duration, Instant};

fn main() {
    let mut master_serial_port = serialport::new("/dev/ttyDXL_master_right", 1_000_000)
        .timeout(Duration::from_millis(200))
        .open()
        .expect("Failed to open port");
    let mut puppet_serial_port = serialport::new("/dev/ttyDXL_puppet_right", 1_000_000)
        .timeout(Duration::from_millis(200))
        .open()
        .expect("Failed to open port");

    let io = DynamixelSerialIO::v2();
    xm::sync_write_torque_enable(
        &io,
        puppet_serial_port.as_mut(),
        &[1, 2, 3, 4, 5, 6, 7, 8],
        &[1; 8],
    )
    .expect("Communication error");

    loop {
        let pos = xm::sync_read_present_position(
            &io,
            master_serial_port.as_mut(),
            &[1, 2, 3, 4, 5, 6, 7, 8],
        )
        .expect("Communication error");
        xm::sync_write_goal_position(
            &io,
            puppet_serial_port.as_mut(),
            &[1, 2, 3, 4, 5, 6, 7, 8],
            &pos,
        )
        .expect("Communication error");
    }
}
