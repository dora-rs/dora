## Mujoco Client

This node is a client for a Mujoco simulation.

## YAML Configuration

````YAML
nodes:
  - id: mujoco_client
    path: client.py # modify this to the relative path from the graph file to the client script
    inputs:
      pull_position: dora/timer/millis/10 # pull the present position every 10ms

      # write_goal_position: some goal position from other node
      # end: some end signal from other node
    outputs:
      - position # regarding 'pull_position' input, it will output the position every 10ms
      - end # end signal that can be sent to other nodes (sent when the simulation ends)

    env:
      SCENE: scene.xml # the scene file to be used in the simulation modify this to the relative path from the graph file to the scene file
      CONFIG: config.json # the configuration file for the simulated arm (only retrieve joints names)
````

## Inputs

## Outputs

## License

This library is licensed under the [Apache License 2.0](../../LICENSE).