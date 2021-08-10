# RclRust

## Introduction

This is yet another ROS2 client library written in Rust.  
I have implemented it independent of the ament or colcon.
By using proc-macro to generate message-type and service-type code, crate dependency resolution can now be completed in `cargo`. This was inspired by [rosrust](https://github.com/adnanademovic/rosrust).

## Development environment

- OS: Ubuntu 20.04
- ROS2: Foxy
- rustc: 1.54.0 (stable)

## Supporting features

- Code generation from `.msg`, `.srv`, `.action`
- Loggers
- Publishers/Subscriptions
- Services/Clients
- Timers
- Parameters (without services)

## TODO

- Parameter services/clients
- Actions
- Multithread
- Lifecycles
- More
  - Unit test
  - Documentation
  - Examples (especially with ament)
- CI
- etc...

## I'm not going to support

- Components
  - Is it necessary for Rust?

## Examples

### Prepare

```sh-session
$ git clone git@github.com:rclrust/rclrust.git
$ cd rclrust
$ cargo build
```

### Pub/Sub

Publisher:

```sh-session
$ cargo run --examples publisher
```

Subscription

```sh-session
$ cargo run --examples subscription
```

![out](https://user-images.githubusercontent.com/25898373/128894819-f925b31f-d814-4046-a328-68bfe854d03b.gif)

For more examples, see https://github.com/rclrust/rclrust/tree/main/rclrust/examples.

## The other ROS2 clients written in Rust

- [ros2_rust](https://github.com/ros2-rust/ros2_rust)
- [r2r](https://github.com/sequenceplanner/r2r)
