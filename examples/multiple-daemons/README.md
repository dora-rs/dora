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

