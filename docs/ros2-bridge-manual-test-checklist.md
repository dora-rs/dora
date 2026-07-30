# ROS2 Bridge Manual Test Checklist[PASSED]

This checklist validates the existing ROS2 bridge examples without adding new
examples. Run the default-domain pass first, then repeat with `ROS_DOMAIN_ID=23`
if needed.

## Common Setup

Clean up possible leftover processes:

```bash
pkill -f 'examples_rclcpp_minimal_service' || true
pkill -f 'examples_rclcpp_minimal_action_server' || true
pkill -f 'examples_rclcpp_minimal_publisher' || true
pkill -f 'turtlesim_node' || true
pkill -f 'dora-ros2-bridge-node' || true
```

For default domain, run in every terminal:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
```

For domain 23, run in every terminal instead:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=23
```

## 1. Topic Publish: Dora to ROS2

Terminal 1:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
cargo run -p dora-ros2-bridge --example rust-ros2-dataflow-topic-pub
```

Terminal 2, while Terminal 1 is still running:

```bash
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
ros2 topic list -t
ros2 topic echo /topic
```

Pass criteria:

```text
/topic [std_msgs/msg/String]
data: The N hello from Dora
```

## 2. Topic Subscribe: ROS2 to Dora

Run:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
cargo run -p dora-ros2-bridge --example rust-ros2-dataflow-topic-sub
```

This example starts the ROS2 minimal publisher automatically.

Pass criteria:

```text
received external event: Ok((std_msgs__String { data: "Hello, world! N" }, ...))
```

## 3. Service Client: Dora to ROS2 Service

Terminal 1, start the ROS2 service server:

```bash
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
ros2 run examples_rclcpp_minimal_service service_main
```

Terminal 2, confirm native ROS2 service works:

```bash
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 40}"
```

Expected response:

```text
sum=42
```

Terminal 2, run the Dora service client:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
dora run examples/ros2-bridge/yaml-bridge-service/dataflow-client.yml --stop-after 45s
```

Pass criteria:

```text
Received response: sum = 11
Received response: sum = 22
```

Stop the ROS2 service server in Terminal 1 after the test.

## 4. Service Server: ROS2 Client to Dora Service

Ensure no native `/add_two_ints` server is still running:

```bash
pkill -f 'examples_rclcpp_minimal_service' || true
```

Terminal 1, start the Dora service server:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
dora run examples/ros2-bridge/yaml-bridge-service/dataflow-server.yml
```

Terminal 2, while Terminal 1 is running:

```bash
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
ros2 service list -t
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 40}"
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 42, b: 40}"
```

Pass criteria in Terminal 2:

```text
sum=42
sum=82
```

Pass criteria in Terminal 1:

```text
Request: 2 + 40 = 42
Request: 42 + 40 = 82
```

Stop Terminal 1 after the test.

## 5. Action Client: Dora to ROS2 Action Server

Terminal 1, start the ROS2 action server:

```bash
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
ros2 run examples_rclcpp_minimal_action_server action_server_member_functions
```

Terminal 2, confirm native ROS2 action works:

```bash
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
ros2 action list -t
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}" --feedback
```

Expected result:

```text
Goal finished with status: SUCCEEDED
```

Terminal 2, run the Dora action client:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
dora run examples/ros2-bridge/yaml-bridge-action/dataflow.yml --stop-after 45s
```

Pass criteria:

```text
ROS2 action service connected: /fibonacci/_action/send_goal
ROS2 action service connected: /fibonacci/_action/get_result
ROS2 action service connected: /fibonacci/_action/cancel_goal
Sending Fibonacci goal: order=5
Feedback: sequence=...
Result: sequence=...
```

Stop the ROS2 action server in Terminal 1 after the test.

## 6. Action Server: ROS2 Client to Dora Action Server

Ensure no native ROS2 action server is still running:

```bash
pkill -f 'examples_rclcpp_minimal_action_server' || true
```

Terminal 1, start the Dora action server:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
dora run examples/ros2-bridge/yaml-bridge-action-server/dataflow.yml
```

Terminal 2, while Terminal 1 is running:

```bash
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
ros2 action list -t
ros2 action info /fibonacci -t
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" --feedback
```

Pass criteria in Terminal 2:

```text
/fibonacci [example_interfaces/action/Fibonacci]
Action servers: 1
Goal accepted
Feedback:
    sequence:
Result:
    sequence:
Goal finished with status: SUCCEEDED
```

Pass criteria in Terminal 1:

```text
Received Fibonacci goal: order=5
Feedback: sequence=...
Result: sequence=...
```

Stop Terminal 1 after the test.

## 7. TurtleSim Example

The Rust turtle example starts these ROS2 peers automatically:

- `examples_rclcpp_minimal_service/service_main`
- `turtlesim/turtlesim_node`

Run:

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
unset ROS_DOMAIN_ID
cargo run -p dora-ros2-bridge --example rust-ros2-dataflow
```

Pass criteria:

```text
turtlesim window appears
turtle moves
Dora terminal has no panic, timeout, or bridge error
```

## Repeat With ROS_DOMAIN_ID=23

To repeat any test with domain 23, replace every:

```bash
unset ROS_DOMAIN_ID
```

with:

```bash
export ROS_DOMAIN_ID=23
```

All terminals participating in the same test must use the same domain.

## Final Cleanup

After testing, check for leftover processes:

```bash
pgrep -af 'dora-ros2-bridge-node|examples_rclcpp_minimal_service|examples_rclcpp_minimal_action_server|examples_rclcpp_minimal_publisher|turtlesim_node' || true
```

Stop leftovers if needed:

```bash
pkill -f 'examples_rclcpp_minimal_service' || true
pkill -f 'examples_rclcpp_minimal_action_server' || true
pkill -f 'examples_rclcpp_minimal_publisher' || true
pkill -f 'turtlesim_node' || true
pkill -f 'dora-ros2-bridge-node' || true
```
