# RclRust

<img src="https://user-images.githubusercontent.com/25898373/131146249-36f349ba-ce33-462d-89f8-40bfa1a9899f.png" width="200px" alt="rclrust's logo"/>

[![rclrust](https://img.shields.io/crates/v/rclrust.svg)](https://crates.io/crates/rclrust)
[![codecov](https://codecov.io/gh/rclrust/rclrust/branch/main/graph/badge.svg)](https://codecov.io/gh/rclrust/rclrust)
[![dependency status](https://deps.rs/repo/github/rclrust/rclrust/status.svg)](https://deps.rs/repo/github/rclrust/rclrust)
[![Rust 1.53](https://img.shields.io/badge/rust-1.53+-blue.svg)](https://blog.rust-lang.org/2021/06/17/Rust-1.53.0.html)
[![Apache-2.0](https://img.shields.io/github/license/rclrust/rclrust)](https://github.com/rclrust/rclrust/blob/main/LICENSE)

<table>
  <thead>
    <tr>
      <th>Target</th>
      <th>OS</th>
      <th>EOL</th>
      <th>CI Status</th>
      <th>Document</th>
      <th>Remarks</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Foxy</td>
      <td>Ubuntu 20.04</td>
      <td>May 2023</td>
      <td><a href="https://github.com/rclrust/rclrust/actions/workflows/foxy.yaml" alt="Foxy CI status">
        <img src="https://github.com/rclrust/rclrust/actions/workflows/foxy.yaml/badge.svg?branch=main"/>
      </a></td>
      <td><a href="https://rclrust.github.io/rclrust/foxy/main/rclrust/index.html" alt="Foxy document">
        <img src="https://github.com/rclrust/rclrust/actions/workflows/doc.yaml/badge.svg"/>
      </a></td>
      <td>-</td>
    </tr>
    <tr>
      <td>Galactic</td>
      <td>Ubuntu 20.04</td>
      <td>Nov 2022</td>
      <td><a href="https://github.com/rclrust/rclrust/actions/workflows/galactic.yaml" alt="Galactic CI status">
        <img src="https://github.com/rclrust/rclrust/actions/workflows/galactic.yaml/badge.svg?branch=main"/>
      </a></td>
      <td><a href="https://rclrust.github.io/rclrust/galactic/main/rclrust/index.html" alt="Galactic document">
        <img src="https://github.com/rclrust/rclrust/actions/workflows/doc.yaml/badge.svg"/>
      </a></td>
      <td>-</td>
    </tr>
    <tr>
      <td>Rolling</td>
      <td>Ubuntu 20.04</td>
      <td>-</td>
      <td><a href="https://github.com/rclrust/rclrust/actions/workflows/rolling.yaml" alt="Rolling CI status">
        <img src="https://github.com/rclrust/rclrust/actions/workflows/rolling.yaml/badge.svg?branch=main"/>
      </a></td>
      <td><a href="https://rclrust.github.io/rclrust/rolling/main/rclrust/index.html" alt="Rolling document">
        <img src="https://github.com/rclrust/rclrust/actions/workflows/doc.yaml/badge.svg"/>
      </a></td>
      <td>For development</td>
    </tr>
  </tbody>
</table>

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

## Notice

The icon of RclRust was created by combinating and modifing the following materials.

- [ros.svg](https://github.com/ros-infrastructure/artwork/blob/master/orgunits/ros.svg) © ROS (Licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/))
- [Gear-icon.png](https://commons.wikimedia.org/wiki/File:Gear-icon.png) (Licensed under [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/deed.en))

## The other ROS2 clients written in Rust

- [ros2_rust](https://github.com/ros2-rust/ros2_rust)
- [r2r](https://github.com/sequenceplanner/r2r)
