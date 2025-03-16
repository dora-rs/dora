use dora_node_api::{dora_core::config::DataId, DoraNode, IntoArrow, MetadataParameters};
use eyre::{Context, Result};
use rustypot::{device::xm, DynamixelSerialIO};
use serialport::SerialPort;
use std::{
    sync::mpsc,
    thread::JoinHandle,
    time::{Duration, Instant},
};
const MAX_MASTER_GRIPER: f64 = 0.7761942786701344;
const MAX_PUPPET_GRIPER: f64 = 1.6827769243105486;

const MIN_MASTER_GRIPER: f64 = -0.12732040539450828;
const MIN_PUPPET_GRIPER: f64 = 0.6933593161243099;

use clap::Parser;

/// Simple aloha teleop program for recording data
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short, long, default_value = "/dev/ttyDXL_master_left")]
    master_left_path: String,

    #[arg(short, long, default_value = "/dev/ttyDXL_puppet_left")]
    puppet_left_path: String,

    #[arg(short, long, default_value = "/dev/ttyDXL_master_right")]
    master_right_path: Option<String>,

    #[arg(short, long, default_value = "/dev/ttyDXL_puppet_right")]
    puppet_right_path: Option<String>,
}

enum State {
    Position(Vec<f64>),
    Velocity(Vec<f64>),
    Current(Vec<u16>),
    GoalPosition(Vec<f64>),
}
#[derive(Clone, Copy)]
enum Side {
    Left,
    Right,
}

fn main_multithreaded(
    io: DynamixelSerialIO,
    mut master_serial_port: Box<dyn SerialPort>,
    mut puppet_serial_port: Box<dyn SerialPort>,
    side: Side,
    tx_dora: mpsc::Sender<(Side, State)>,
) -> Result<JoinHandle<()>> {
    let (tx, rx) = mpsc::channel();
    let tx_dora_read = tx_dora.clone();
    std::thread::spawn(move || loop {
        let now = Instant::now();
        let pos = xm::sync_read_present_position(
            &io,
            master_serial_port.as_mut(),
            &[1, 2, 4, 6, 7, 8, 9], // 3 and 5 are skipped as thery share the same position as 2 and 4
        )
        .expect("Read Communication error");
        let radians: Vec<f64> = pos.iter().map(|&x| xm::conv::pos_to_radians(x)).collect();
        tx.send((now, radians.clone())).unwrap();
        tx_dora.send((side, State::Position(radians))).unwrap();
        let vel = xm::sync_read_present_velocity(
            &io,
            master_serial_port.as_mut(),
            &[1, 2, 4, 6, 7, 8, 9], // 3 and 5 are skipped as thery share the same position as 2 and 4
        )
        .expect("Read Communication error");
        let rad_per_sec: Vec<f64> = vel
            .iter()
            .map(|&x| xm::conv::abs_speed_to_rad_per_sec(x))
            .collect();
        tx_dora.send((side, State::Velocity(rad_per_sec))).unwrap();
        let load =
            xm::sync_read_present_current(&io, master_serial_port.as_mut(), &[1, 2, 4, 6, 7, 8, 9]) // 3 and 5 are skipped as thery share the same position as 2 and 4
                .expect("Read Communication error");
        tx_dora.send((side, State::Current(load))).unwrap();
    });

    let io = DynamixelSerialIO::v2();
    let join = std::thread::spawn(move || {
        while let Ok((_now, mut pos)) = rx.recv() {
            pos[6] = (pos[6] - MIN_MASTER_GRIPER) * (MAX_PUPPET_GRIPER - MIN_PUPPET_GRIPER)
                / (MAX_MASTER_GRIPER - MIN_MASTER_GRIPER)
                + MIN_PUPPET_GRIPER;
            tx_dora_read
                .send((side, State::GoalPosition(pos.clone())))
                .unwrap();
            pos.insert(2, pos[1]);
            pos.insert(4, pos[3]);
            let pos: Vec<u32> = pos.iter().map(|&x| xm::conv::radians_to_pos(x)).collect();
            // Compute linear interpolation for gripper as input and output range mismatch
            xm::sync_write_goal_position(
                &io,
                puppet_serial_port.as_mut(),
                &[1, 2, 3, 4, 5, 6, 7, 8, 9],
                &pos,
            )
            .expect("Write Communication error");
            // println!("elapsed time: {:?}", now.elapsed());
        }
    });

    Ok(join)
}

fn main() -> Result<()> {
    let args = Args::parse();
    let (tx_dora, rx_dora) = mpsc::channel();
    // right arm
    let join_right = if let (Some(master_right_path), Some(puppet_right_path)) =
        (args.master_right_path.clone(), args.puppet_right_path)
    {
        let master_right_serial_port = serialport::new(master_right_path, 1000000)
            .timeout(Duration::from_millis(2))
            .open()
            .context("Failed to open port")?;
        let mut puppet_right_serial_port = serialport::new(puppet_right_path, 1000000)
            .timeout(Duration::from_millis(2))
            .open()
            .context("Failed to open port")?;

        let io = DynamixelSerialIO::v2();
        xm::sync_write_torque_enable(
            &io,
            puppet_right_serial_port.as_mut(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 9],
            &[1; 9],
        )
        .expect("Communication error");
        // Set operating mode to current based position control to not overload the gripper
        xm::sync_write_operating_mode(&io, puppet_right_serial_port.as_mut(), &[9], &[5])
            .expect("Communication error");
        Some(main_multithreaded(
            io,
            master_right_serial_port,
            puppet_right_serial_port,
            Side::Right,
            tx_dora.clone(),
        )?)
    } else {
        None
    };

    // Left arm
    let master_left_serial_port = serialport::new(args.master_left_path, 1000000)
        .timeout(Duration::from_millis(2))
        .open()
        .context("Failed to open port")?;
    let mut puppet_left_serial_port = serialport::new(args.puppet_left_path, 1000000)
        .timeout(Duration::from_millis(2))
        .open()
        .context("Failed to open port")?;
    let io = DynamixelSerialIO::v2();
    xm::sync_write_torque_enable(
        &io,
        puppet_left_serial_port.as_mut(),
        &[1, 2, 3, 4, 5, 6, 7, 8, 9],
        &[1; 9],
    )
    .expect("Communication error");

    // Set operating mode to current based position control to not overload the gripper
    xm::sync_write_operating_mode(&io, puppet_left_serial_port.as_mut(), &[9], &[5])
        .expect("Communication error");

    let join_left = main_multithreaded(
        io,
        master_left_serial_port,
        puppet_left_serial_port,
        Side::Left,
        tx_dora,
    )?;

    if std::env::var("DORA_NODE_CONFIG").is_ok() {
        let (mut node, mut events) = DoraNode::init_from_env()?;
        let mut pos_right = Vec::new();
        let mut vel_right = Vec::new();
        let mut current_right = Vec::new();
        let mut goal_right = Vec::new();
        while let Ok((side, target)) = rx_dora.recv() {
            match side {
                Side::Left => {
                    let parameters = MetadataParameters::default();

                    // Skip if we don't have any data from the right arm
                    if args.master_right_path.is_some() && pos_right.is_empty() {
                        continue;
                    }
                    match target {
                        State::Position(mut pos) => {
                            pos.extend_from_slice(&pos_right);
                            let output = DataId::from("puppet_position".to_owned());
                            node.send_output(output.clone(), parameters, pos.into_arrow())?;
                        }
                        State::Velocity(mut vel) => {
                            vel.extend_from_slice(&vel_right);
                            let output = DataId::from("puppet_velocity".to_owned());
                            node.send_output(output.clone(), parameters, vel.into_arrow())?;
                        }
                        State::Current(mut load) => {
                            load.extend_from_slice(&current_right);
                            let output = DataId::from("puppet_current".to_owned());
                            node.send_output(output.clone(), parameters, load.into_arrow())?;
                        }
                        State::GoalPosition(mut pos) => {
                            pos.extend_from_slice(&goal_right);
                            let output = DataId::from("puppet_goal_position".to_owned());
                            node.send_output(output.clone(), parameters, pos.into_arrow())?;
                        }
                    }
                    if events.recv_timeout(Duration::from_nanos(100)).is_none() {
                        println!("Events channel finished");
                        break;
                    }
                }
                Side::Right => match target {
                    State::Position(pos) => {
                        pos_right = pos;
                    }
                    State::Velocity(vel) => {
                        vel_right = vel;
                    }
                    State::Current(load) => {
                        current_right = load;
                    }
                    State::GoalPosition(pos) => {
                        goal_right = pos;
                    }
                },
            }
        }
    } else {
        join_left.join().unwrap();
        join_right.map(|join| join.join().unwrap());
    };
    Ok(())
}
