# Example for connecting multiple daemons

Dora supports splitting a dataflow onto multiple daemon instances, which typically run on different machines.
In this example, we spawn two daemons instances `A` and `B`, which connect to the same coordinator.
One node of the dataflow is run on daemon `B`, the other two nodes are run on daemon `A`.

Normally, daemon `A` and `B` would run on different machines.
To keep this example self-contained, we run them on the same machine here.

## Quick run

To run the example like we do on CI, run `cargo run --example multiple-daemons`.
This command will build everything, launch a test coordinator and two daemon instances, and then run the `dataflow.yml` there.
See the next section for more detailed run instructions.

## Manual run

Execute the following steps in this directory:

- Build all nodes of the dataflow by running `dora build dataflow.yml`.
- Start a coordinator by running `dora coordinator`.
  - If you want more log output, run `RUST_LOG=debug dora coordinator` instead.
- In a new terminal, start the first daemon instance by running `dora daemon --machine-id A`.
  - The `--machine-id A` argument assigns an identifier to the daemon instance. This is then used in the `dataflow.yml` file to assign nodes to daemon instances.
  - Again, you can specify a `RUST_LOG` env variable for more output if you like
- In a third terminal, start the second daemon instance by running `dora daemon --machine-id B --local-listen-port 53292`.
  - We set a different `--machine-id` for the second daemon instance. In the `dataflow.yml` file you see the nodes that are assigned to machine `B`.
  - The `--local-listen-port` argument is required because the first daemon instance listens on the standard port number already.
  - As above, you can specify a `RUST_LOG` env variable for more output if you like
- Start the dataflow through `dora start dataflow.yml`

# Usage Across Multiple Machines

The steps below start each daemon by hand. For a managed cluster — machine list
in a `cluster.yml`, SSH lifecycle commands, label-based scheduling, systemd
services — see the [Distributed Deployment Guide](../../docs/distributed-deployment.md),
which also documents how each daemon picks the address it advertises to the
others.

1. Start the dora coordinator on some machine that is reachable by IP by all other machines
    ```bash
    dora coordinator
    ```
    - If you plan to run dora in the cloud, the machine that runs the `dora coordinator` needs to have a public IP.
    - Set a `RUST_LOG` environment variable to get more output on the command line. For example, `RUST_LOG=debug dora coordinator` reports when daemons connect and when dataflow start commands are received.
2. Start dora daemon instances on all machines that should run parts of the dataflow.
    - If all daemons and the coordinator are **in the same network** (e.g. same WiFi), use the following command:
      ```bash
      RUST_LOG=debug dora daemon --coordinator-addr <IP> --machine-id <MACHINE_ID>
      ```
      Replace `<IP>` with the (public) IP address of the coordinator. Replace `<MACHINE_ID>` with some unique identifier for each machine. This machine ID will be used to assign nodes to machines in the `dataflow.yml` file.

      You can **omit the `--machine-id` argument on one machine**. This will be the default machine for nodes that don't specify deploy information.

      The `RUST_LOG=debug` environment variable is optional. It enables some interesting debug output about the dataflow run.
    - If daemons/coordinator are in **different networks** (e.g. behind a [NAT](https://en.wikipedia.org/wiki/Network_address_translation)), put the machines on a **mesh VPN** such as [Tailscale](https://tailscale.com/), [WireGuard](https://www.wireguard.com/), or [Nebula](https://github.com/slackhq/nebula). This is the recommended setup: once every machine has a tunnel address, the machines are mutually dialable and dora needs no extra zenoh configuration. Each daemon binds the local address that routes toward the coordinator — the tunnel address, when the coordinator is reached through the tunnel — and advertises exactly that to the other daemons.
      ```bash
      RUST_LOG=debug dora daemon --coordinator-addr <COORDINATOR_TUNNEL_IP> --machine-id <MACHINE_ID> \
        --zenoh-peer tcp/<COORDINATOR_TUNNEL_IP>:5456
      ```
      A mesh VPN carries no multicast, so daemons cannot find each other by scouting. `--zenoh-peer` gives them an explicit rendezvous instead: pass the *same* endpoint to every daemon, and the first one to bind it serves as the rendezvous for the others. On a multi-homed machine that would otherwise advertise an interface the other daemons cannot reach, name the address explicitly with `--zenoh-listen <TUNNEL_IP>`.

      A [zenoh router](https://zenoh.io/docs/getting-started/deployment/#zenoh-router) is **not** a substitute for this. Since zenoh 1.9, peers no longer relay for each other, and peers that sit under one router in a single region are no exception: two NAT'd daemons will discover each other through a `zenohd` and still exchange nothing. A router helps only where the daemons are mutually dialable anyway (it then supplies discovery, which `--zenoh-peer` also does), or where the peers are split into separate router subregions.

      If you do run your own [`zenohd`](https://zenoh.io/docs/getting-started/installation/) instances, point dora at them with a `ZENOH_CONFIG` environment variable:
      ```bash
      ZENOH_CONFIG=<ZENOH_CONFIG_FILE_PATH> RUST_LOG=debug dora daemon --coordinator-addr <IP> --machine-id <MACHINE_ID>
      ```
      Replace `<ZENOH_CONFIG_FILE_PATH>` with the path to a [zenoh configuration file](https://zenoh.io/docs/manual/configuration/#configuration-files) that lists the corresponding `zenohd` instance(s) under `connect.endpoints`. Note that this variable replaces dora's zenoh configuration **wholesale**, including the direct node-to-node links the daemon plans for same-machine communication — so same-machine traffic is relayed through the router too, which is slower and, if the router runs on another host, sends that traffic off the machine and back.

3. In your `dataflow.yml` file, add an `deploy` key to all nodes that should not run on the default machine:
    ```yml
    - id: example-node
      deploy:
        machine: laptop        # this should match the <MACHINE_ID> of one daemon
      path: /home/user/dora/target/debug/rust-dataflow-example-node
      inputs: [...]
      outputs: [...]
    ```
    The `path` field should be an absolute path because we don't have a clear default for the working directory on remote nodes yet.
4. Build all the nodes on their target machines, or build them locally and copy them over.

  Dora does **not** do any deployment of executables yet. You need to copy the executables/Python code yourself.
5. Use the CLI to start the dataflow:
  ```bash
  RUST_LOG=debug dora start --coordinator-addr <IP> dataflow.yml
  ```

  Replace `<IP>` with the (public) IP address of the coordinator. The `--coordinator-addr` argument is optional if the coordinator is running on the same machine as the CLI.

  As above, the `RUST_LOG=debug` env variable is optional. You can omit it if you're not interested in the additional output.

## Zenoh Topics

Dora publishes outputs destined for remote nodes on the following zenoh topic names:

```
dora/{network_id}/{dataflow_id}/output/{node_id}/{output_id}
```

- The `{network_id}` is always `default` for now. In the future, we plan to make it configurable to allow operating multiple separate dora networks within the same WiFi.
- The `{dataflow_id}` is the UUID assigned to the dataflow instance on start. This UUID will be different on every start. (If you want to debug your dataflow using a separate zenoh client and you don't care about the dataflow instance, you can use a `*` wildcard when subscribing.)
- The `{node_id}` is the `id` field of the node specified in the `dataflow.yml`.
- The `{output_id}` is the name of the node output as specified in the `dataflow.yml`.
