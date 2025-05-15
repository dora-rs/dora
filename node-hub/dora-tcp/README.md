# dora-tcp

Basic TCP Implementation for streaming data to a TCP Destination. 


## Getting Started
Copy the node to your dora project and build it


Example:
- Streaming drive commands to a Unity simulation hosting a TCP server.

Example dataflow.yml
```shell
nodes:
  - id: drive_director
    build: cargo build -p drive_director
    path: target/debug/drive_director
    inputs:
      tick: dora/timer/millis/100
    outputs:
      - command
  - id: dora-tcp
    build: cargo build -p dora-tcp
    path: target/debug/dora-tcp
    inputs:
      command: drive_director/command
```

## Input definition

- command: String

Note: I used a colon to separate the commands ie drive:10 but you can modify as needed.