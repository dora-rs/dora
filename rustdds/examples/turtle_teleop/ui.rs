#![cfg(unix)]

use std::{io::Write, os::unix::io::AsRawFd};

#[allow(unused_imports)]
use log::{debug, error, info, warn};
use mio_06::{unix::EventedFd, Events, Poll, PollOpt, Ready, Token};
use mio_extras::channel as mio_channel;
use termion::{event::Key, input::TermRead, AsyncReader};

use crate::{Pose, Twist, Vector3};

#[derive(Debug)]
pub enum RosCommand {
  StopEventLoop,
  TurtleCmdVel { twist: Twist },
}

// Define turtle movement commands as Twist values
const MOVE_FORWARD: Twist = Twist {
  linear: Vector3 {
    x: 2.0,
    ..Vector3::ZERO
  },
  angular: Vector3::ZERO,
};

const MOVE_BACKWARD: Twist = Twist {
  linear: Vector3 {
    x: -2.0,
    ..Vector3::ZERO
  },
  angular: Vector3::ZERO,
};

const ROTATE_LEFT: Twist = Twist {
  linear: Vector3::ZERO,
  angular: Vector3 {
    z: 2.0,
    ..Vector3::ZERO
  },
};

const ROTATE_RIGHT: Twist = Twist {
  linear: Vector3::ZERO,
  angular: Vector3 {
    z: -2.0,
    ..Vector3::ZERO
  },
};

pub struct UiController {
  poll: Poll,
  stdout: std::io::Stdout,
  async_reader: termion::input::Events<AsyncReader>,
  command_sender: mio_channel::SyncSender<RosCommand>,
  readback_receiver: mio_channel::Receiver<Twist>,
  pose_receiver: mio_channel::Receiver<Pose>,
}

impl UiController {
  const KEYBOARD_CHECK_TOKEN: Token = Token(0);
  const READBACK_TOKEN: Token = Token(1);
  const POSE_TOKEN: Token = Token(2);

  pub fn new(
    stdout: std::io::Stdout,
    command_sender: mio_channel::SyncSender<RosCommand>,
    readback_receiver: mio_channel::Receiver<Twist>,
    pose_receiver: mio_channel::Receiver<Pose>,
  ) -> UiController {
    let poll = Poll::new().unwrap();
    let async_reader = termion::async_stdin().events();

    UiController {
      poll,
      stdout,
      async_reader,
      command_sender,
      readback_receiver,
      pose_receiver,
    }
  }

  pub fn start(&mut self) {
    self
      .poll
      .register(
        &EventedFd(&std::io::stdin().lock().as_raw_fd()),
        UiController::KEYBOARD_CHECK_TOKEN,
        Ready::readable(),
        PollOpt::level(),
      )
      .unwrap();

    self
      .poll
      .register(
        &self.readback_receiver,
        UiController::READBACK_TOKEN,
        Ready::readable(),
        PollOpt::edge(),
      )
      .unwrap();
    self
      .poll
      .register(
        &self.pose_receiver,
        UiController::POSE_TOKEN,
        Ready::readable(),
        PollOpt::edge(),
      )
      .unwrap();

    // clearing screen
    write!(
      self.stdout,
      "{}{}Press q to quit, cursor keys to control turtle.",
      termion::clear::All,
      termion::cursor::Goto(1, 1)
    )
    .unwrap();
    self.stdout.flush().unwrap();

    loop {
      write!(self.stdout, "{}", termion::cursor::Goto(1, 1)).unwrap();
      self.stdout.flush().unwrap();

      let mut events = Events::with_capacity(100);
      self.poll.poll(&mut events, None).unwrap();

      for event in events.iter() {
        if event.token() == UiController::KEYBOARD_CHECK_TOKEN {
          // a small wait here to allow the termion input mechnism to react.
          // Still some keyboard presses are missed. What are we doing wrong here?
          std::thread::sleep(std::time::Duration::from_millis(10));
          while let Some(Ok(termion::event::Event::Key(key))) = &self.async_reader.next() {
            write!(
              self.stdout,
              "{}{}{:?}",
              termion::cursor::Goto(1, 2),
              termion::clear::CurrentLine,
              key,
            )
            .unwrap();
            info!("key: {:?}", key);
            match key {
              Key::Char('q') => {
                debug!("Quit.");
                self.send_command(RosCommand::StopEventLoop);
                return; // stop loop
              }
              Key::Up => {
                debug!("Move left.");
                let twist = MOVE_FORWARD;
                self.print_sent_turtle_cmd_vel(&twist);
                self.send_command(RosCommand::TurtleCmdVel { twist })
              }
              Key::Right => {
                debug!("Move right.");
                let twist = ROTATE_RIGHT;
                self.print_sent_turtle_cmd_vel(&twist);
                self.send_command(RosCommand::TurtleCmdVel { twist })
              }
              Key::Down => {
                debug!("Rotate down.");
                let twist = MOVE_BACKWARD;
                self.print_sent_turtle_cmd_vel(&twist);
                self.send_command(RosCommand::TurtleCmdVel { twist })
              }
              Key::Left => {
                debug!("Rotate left.");
                let twist = ROTATE_LEFT;
                self.print_sent_turtle_cmd_vel(&twist);
                self.send_command(RosCommand::TurtleCmdVel { twist })
              }
              _ => (),
            }
          }
        } else if event.token() == UiController::READBACK_TOKEN {
          while let Ok(twist) = self.readback_receiver.try_recv() {
            write!(
              self.stdout,
              "{}{}Read Turtle cmd_vel {:?}",
              termion::cursor::Goto(1, 6),
              termion::clear::CurrentLine,
              twist
            )
            .unwrap();
          }
        } else if event.token() == UiController::POSE_TOKEN {
          while let Ok(pose) = self.pose_receiver.try_recv() {
            write!(
              self.stdout,
              "{}{}Turtle pose {:?}",
              termion::cursor::Goto(1, 8),
              termion::clear::CurrentLine,
              pose
            )
            .unwrap();
          }
        } else {
          error!("What is this? {:?}", event.token())
        }
      }
    }
  }

  fn send_command(&self, command: RosCommand) {
    self
      .command_sender
      .try_send(command)
      .unwrap_or_else(|e| error!("UI: Failed to send command {e:?}"))
  }

  fn print_sent_turtle_cmd_vel(&mut self, twist: &Twist) {
    write!(
      self.stdout,
      "{}{}Sent Turtle cmd_vel {:?}",
      termion::cursor::Goto(1, 4),
      termion::clear::CurrentLine,
      twist
    )
    .unwrap();
  }
}
