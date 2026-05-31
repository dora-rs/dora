import datetime
import typing

import pyarrow

import dora

@typing.final
class Enum:
    """Create a collection of name/value pairs.

    Example enumeration:

    >>> class Color(Enum):
    ...     RED = 1
    ...     BLUE = 2
    ...     GREEN = 3

    Access them by:

    - attribute access:

    >>> Color.RED
    <Color.RED: 1>

    - value lookup:

    >>> Color(1)
    <Color.RED: 1>

    - name lookup:

    >>> Color['RED']
    <Color.RED: 1>

    Enumerations can be iterated over, and know how many members they have:

    >>> len(Color)
    3

    >>> list(Color)
    [<Color.RED: 1>, <Color.BLUE: 2>, <Color.GREEN: 3>]

    Methods can be added to enumerations, and members can have their own
    attributes -- see the documentation for details."""

    @staticmethod
    def __contains__(value: typing.Any) -> bool:
        """Return True if `value` is in `cls`.

        `value` is in `cls` if:
        1) `value` is a member of `cls`, or
        2) `value` is the value of one of the `cls`'s members."""

    @staticmethod
    def __getitem__(name: typing.Any) -> typing.Any:
        """Return the member matching `name`."""

    @staticmethod
    def __iter__() -> typing.Any:
        """Return members in definition order."""

    @staticmethod
    def __len__() -> int:
        """Return the number of members (no aliases)"""

@typing.final
class Node:
    """The custom node API lets you integrate `dora` into your application.
    It allows you to retrieve input and send output in any fashion you want.

    Use with:

    ```python
    from dora import Node

    node = Node()
    ```"""

    def __init__(self, node_id: str = None, daemon_port: int = None) -> None:
        """The custom node API lets you integrate `dora` into your application.
        It allows you to retrieve input and send output in any fashion you want.

        Creating a Node automatically bridges Python's ``logging`` module to the
        dora daemon. After ``Node()`` is created, ``logging.info()`` etc. produce
        structured log entries that work with ``min_log_level``, ``send_logs_as``,
        and ``dora/logs`` subscribers. No extra configuration needed.

        Use with:

        ```python
        from dora import Node

        node = Node()
        ```

        For a dynamic node connecting to a daemon on a custom port:

        ```python
        node = Node("my-node", daemon_port=6789)
        ```"""

    def log(
        self,
        level: str,
        message: str,
        target: typing.Optional[str] = None,
        fields: typing.Optional[typing.Dict[str, str]] = None,
    ) -> None:
        """Send a structured log message.

        Outputs a JSONL line to stdout that the daemon parses automatically.
        Works with ``min_log_level`` filtering and ``send_logs_as`` routing.

        :param level: Log level string (error, warn, info, debug, trace)
        :param message: The log message
        :param target: Optional target/module path
        :param fields: Optional key-value pairs for structured context"""

    def log_error(self, message: str) -> None:
        """Log an error message. Shorthand for ``node.log("error", message)``."""

    def log_warn(self, message: str) -> None:
        """Log a warning message. Shorthand for ``node.log("warn", message)``."""

    def log_info(self, message: str) -> None:
        """Log an info message. Shorthand for ``node.log("info", message)``."""

    def log_debug(self, message: str) -> None:
        """Log a debug message. Shorthand for ``node.log("debug", message)``."""

    def log_trace(self, message: str) -> None:
        """Log a trace message. Shorthand for ``node.log("trace", message)``."""

    def dataflow_descriptor(self) -> dict:
        """Returns the full dataflow descriptor that this node is part of.

        This method returns the parsed dataflow YAML file."""

    def dataflow_id(self) -> str:
        """Returns the dataflow id."""

    def is_restart(self) -> bool:
        """Returns True if this node was restarted after a previous exit or failure.

        Nodes can use this to decide whether to restore saved state or start fresh."""

    def restart_count(self) -> int:
        """Returns how many times this node has been restarted.

        Returns 0 on the first run, 1 after the first restart, etc."""

    def timestamp(self) -> "datetime.datetime":
        """Returns the current timestamp from the node's Hybrid Logical Clock
        as a UTC datetime object."""

    def merge_external_events(self, subscription: dora.Ros2Subscription) -> None:
        """Merge an external event stream with dora main loop.
        This currently only work with ROS2."""

    def next(self, timeout: float = None) -> dict:
        """`.next()` gives you the next input that the node has received.
        It blocks until the next event becomes available.
        You can use timeout in seconds to return if no input is available.
        It will return `None` when all senders has been dropped.

        ```python
        event = node.next()
        ```

        You can also iterate over the event stream with a loop

        ```python
        for event in node:
        match event["type"]:
        case "INPUT":
        match event["id"]:
        case "image":
        ```"""

    def node_config(self) -> dict:
        """Returns the node configuration."""

    def recv_async(self, timeout: float = None) -> dict:
        """`.recv_async()` gives you the next input that the node has received asynchronously.
        It does not blocks until the next event becomes available.
        You can use timeout in seconds to return if no input is available.
        It will return an Error if the timeout is reached.
        It will return `None` when all senders has been dropped.

        warning::
        This feature is experimental as pyo3 async (rust-python FFI) is still in development.

        ```python
        event = await node.recv_async()
        ```

        You can also iterate over the event stream with a loop"""

    def send_output(
        self, output_id: str, data: pyarrow.Array, metadata: dict = None
    ) -> None:
        """`send_output` send data from the node.

        ```python
        Args:
        output_id: str,
        data: pyarrow.Array,
        metadata: Option[Dict],
        ```

        ex:

        ```python
        node.send_output("string", b"string", {"open_telemetry_context": "7632e76"})
        ```"""

    def __iter__(self) -> typing.Any:
        """Implement iter(self)."""

    def __next__(self) -> typing.Any:
        """Implement next(self)."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2Context:
    """ROS2 Context holding all messages definition for receiving and sending messages to ROS2.

    By default, Ros2Context will use env `AMENT_PREFIX_PATH` to search for message definition.

    AMENT_PREFIX_PATH folder structure should be the following:

    - For messages: <namespace>/msg/<name>.msg
    - For services: <namespace>/srv/<name>.srv

    You can also use `ros_paths` if you don't want to use env variable.

    warning::
    Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change.

    ```python
    context = Ros2Context()
    ```"""

    def __init__(self, ros_paths: typing.List[str] = None) -> None:
        """ROS2 Context holding all messages definition for receiving and sending messages to ROS2.

        By default, Ros2Context will use env `AMENT_PREFIX_PATH` to search for message definition.

        AMENT_PREFIX_PATH folder structure should be the following:

        - For messages: <namespace>/msg/<name>.msg
        - For services: <namespace>/srv/<name>.srv

        You can also use `ros_paths` if you don't want to use env variable.

        warning::
        Dora ROS2 bridge functionality is considered **unstable**. It may be changed
        at any point without it being considered a breaking change.

        ```python
        context = Ros2Context()
        ```"""

    def new_node(
        self,
        name: str,
        namespace: str,
        options: dora.Ros2NodeOptions,
        parameters: typing.Optional[dict] = None,
    ) -> dora.Ros2Node:
        """Create a new ROS2 node

        `parameters` is an optional dict of initial ROS2 parameters to declare
        (name -> bool/int/float/str/bytes/list); they are then served via the
        standard ROS2 parameter services and can be read/updated with
        `Ros2Node.get_parameter` / `set_parameter`.

        ```python
        ros2_node = ros2_context.new_node(
        "turtle_teleop",
        "/ros2_demo",
        Ros2NodeOptions(rosout=True),
        parameters={"speed": 1.5},
        )
        ```

        warning::
        Dora ROS2 bridge functionality is considered **unstable**. It may be changed
        at any point without it being considered a breaking change."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2Durability:
    """DDS 2.2.3.4 DURABILITY"""

    def __eq__(self, value: typing.Any) -> bool:
        """Return self==value."""

    def __ge__(self, value: typing.Any) -> bool:
        """Return self>=value."""

    def __gt__(self, value: typing.Any) -> bool:
        """Return self>value."""

    def __int__(self) -> None:
        """int(self)"""

    def __le__(self, value: typing.Any) -> bool:
        """Return self<=value."""

    def __lt__(self, value: typing.Any) -> bool:
        """Return self<value."""

    def __ne__(self, value: typing.Any) -> bool:
        """Return self!=value."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2Liveliness:
    """DDS 2.2.3.11 LIVELINESS"""

    def __eq__(self, value: typing.Any) -> bool:
        """Return self==value."""

    def __ge__(self, value: typing.Any) -> bool:
        """Return self>=value."""

    def __gt__(self, value: typing.Any) -> bool:
        """Return self>value."""

    def __int__(self) -> None:
        """int(self)"""

    def __le__(self, value: typing.Any) -> bool:
        """Return self<=value."""

    def __lt__(self, value: typing.Any) -> bool:
        """Return self<value."""

    def __ne__(self, value: typing.Any) -> bool:
        """Return self!=value."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2Node:
    """ROS2 Node

    warnings::
    - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change.
    - There's a known issue about ROS2 nodes not being discoverable by ROS2
    See: https://github.com/jhelovuo/ros2-client/issues/4"""

    def create_publisher(
        self, topic: dora.Ros2Topic, qos: dora.Ros2QosPolicies = None
    ) -> dora.Ros2Publisher:
        """Create a ROS2 publisher

        ```python
        pose_publisher = ros2_node.create_publisher(turtle_pose_topic)
        ```
        warnings:
        - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
        at any point without it being considered a breaking change."""

    def create_subscription(
        self, topic: dora.Ros2Topic, qos: dora.Ros2QosPolicies = None
    ) -> dora.Ros2Subscription:
        """Create a ROS2 subscription

        ```python
        pose_reader = ros2_node.create_subscription(turtle_pose_topic)
        ```

        warnings:
        - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
        at any point without it being considered a breaking change."""

    def set_parameter(
        self, name: str, value: typing.Union[bool, int, float, str, bytes, list]
    ) -> None:
        """Set a ROS2 parameter (declared via `new_node(parameters=...)`).

        The change is published on `/parameter_events` and is visible to
        `ros2 param get` and rclpy parameter clients."""

    def get_parameter(
        self, name: str
    ) -> typing.Optional[typing.Union[bool, int, float, str, bytes, list]]:
        """Get a ROS2 parameter value, or `None` if it is not set."""

    def list_parameters(self) -> typing.List[str]:
        """List the names of all declared ROS2 parameters."""

    def has_parameter(self, name: str) -> bool:
        """Whether a ROS2 parameter with the given name is declared."""

    def create_topic(
        self, name: str, message_type: str, qos: dora.Ros2QosPolicies
    ) -> dora.Ros2Topic:
        """Create a ROS2 topic to connect to.

        ```python
        turtle_twist_topic = ros2_node.create_topic(
        "/turtle1/cmd_vel", "geometry_msgs/Twist", topic_qos
        )
        ```"""

    def create_service_client(
        self, service_name: str, service_type: str, qos: dora.Ros2QosPolicies
    ) -> dora.Ros2ServiceClient:
        """Create a ROS2 service client.

        Waits (bounded) for a matching service server to become available
        before returning. ROS2 services require **reliable** QoS.

        ```python
        client = ros2_node.create_service_client(
        "/add_two_ints", "example_interfaces/AddTwoInts",
        Ros2QosPolicies(reliable=True),
        )
        response = client.call(pa.array([{"a": 2, "b": 3}]))
        ```"""

    def create_service_server(
        self, service_name: str, service_type: str, qos: dora.Ros2QosPolicies
    ) -> dora.Ros2ServiceServer:
        """Create a ROS2 service server.

        Drive it by polling `take_request` and replying with `send_response`.
        ROS2 services require **reliable** QoS.

        ```python
        server = ros2_node.create_service_server(
        "/add_two_ints", "example_interfaces/AddTwoInts",
        Ros2QosPolicies(reliable=True),
        )
        req = server.take_request(timeout_s=0.1)
        if req is not None:
        request_id, value = req
        args = value[0].as_py()
        server.send_response(request_id, pa.array([{"sum": args["a"] + args["b"]}]))
        ```"""

    def create_action_client(
        self, action_name: str, action_type: str, qos: dora.Ros2QosPolicies
    ) -> dora.Ros2ActionClient:
        """Create a ROS2 action client.

        There is no `wait_for_action_server`; the first `send_goal` simply times
        out (default 30s) if no server is present. ROS2 actions require
        **reliable** QoS.

        ```python
        client = ros2_node.create_action_client(
        "/fibonacci", "example_interfaces/Fibonacci",
        Ros2QosPolicies(reliable=True),
        )
        goal_id = client.send_goal(pa.array([{"order": 5}]))
        status, result = client.take_result(goal_id, timeout_s=30.0)
        ```"""

    def create_action_server(
        self, action_name: str, action_type: str, qos: dora.Ros2QosPolicies
    ) -> dora.Ros2ActionServer:
        """Create a ROS2 action server.

        Drive it by polling `take_goal`, then `send_feedback` / `send_result`
        (and optionally `take_cancel`). Goals are auto-accepted and driven to
        Executing on `take_goal`. ROS2 actions require **reliable** QoS.

        ```python
        server = ros2_node.create_action_server(
        "/fibonacci", "example_interfaces/Fibonacci",
        Ros2QosPolicies(reliable=True),
        )
        goal = server.take_goal(timeout_s=0.1)
        if goal is not None:
        goal_id, value = goal
        server.send_result(goal_id, pa.array([{"sequence": [0, 1, 1]}]), status="succeeded")
        ```"""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2NodeOptions:
    """ROS2 Node Options"""

    def __init__(self, rosout: bool = None) -> None:
        """ROS2 Node Options"""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2Publisher:
    """ROS2 Publisher

    warnings:
    - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change."""

    def publish(self, data: pyarrow.Array) -> None:
        """Publish a message into ROS2 topic.

        Remember that the data format should respect the structure of the ROS2 message using an arrow Structure.

        ex:
        ```python
        gripper_command.publish(
        pa.array(
        [
        {
        "name": "gripper",
        "cmd": np.float32(5),
        }
        ]
        ),
        )
        ```"""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2ServiceClient:
    """ROS2 service client. Create via `Ros2Node.create_service_client`.

    warnings:
    - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change."""

    def call(self, request: pyarrow.Array, timeout_s: float = None) -> pyarrow.Array:
        """Send a request and block until the response arrives.

        The request must match the service's `_Request` structure as an Arrow
        struct (e.g. `pa.array([{"a": 2, "b": 3}])`). Returns the `_Response`
        as a pyarrow array. Raises on timeout (default 30s)."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2ServiceServer:
    """ROS2 service server. Create via `Ros2Node.create_service_server`.

    warnings:
    - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change."""

    def take_request(
        self, timeout_s: float = None
    ) -> typing.Optional[typing.Tuple[int, pyarrow.Array]]:
        """Wait up to `timeout_s` (default 1s) for the next request.

        Returns `(request_id, request_array)`, or `None` if no request arrived
        within the timeout. Pass `request_id` back to `send_response`."""

    def send_response(self, request_id: int, response: pyarrow.Array) -> None:
        """Send the response for a request previously returned by `take_request`."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2ActionClient:
    """ROS2 action client. Create via `Ros2Node.create_action_client`.

    warnings:
    - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change.
    - There is no `wait_for_action_server`; the first `send_goal` times out
    (default 30s) if no server is present.
    - Feedback for concurrent goals shares one subscription; prefer one in-flight
    goal per client."""

    def send_goal(
        self, goal: pyarrow.Array, timeout_s: float = None
    ) -> typing.Optional[str]:
        """Send a goal and block until the server accepts or rejects it.

        The goal must match the action's `_Goal` structure as an Arrow struct
        (e.g. `pa.array([{"order": 5}])`). Returns the goal_id string on accept,
        or `None` on rejection. Raises on timeout (default 30s)."""

    def take_feedback(
        self, goal_id: str, timeout_s: float = None
    ) -> typing.Optional[pyarrow.Array]:
        """Poll up to `timeout_s` (default 1s) for one feedback message.

        Returns the `_Feedback` array, or `None` if none arrived within the
        timeout (poll-again semantics, not an error)."""

    def take_result(
        self, goal_id: str, timeout_s: float = None
    ) -> typing.Optional[typing.Tuple[str, pyarrow.Array]]:
        """Request and block for the terminal result of `goal_id`.

        Returns `(status_str, result_array)` on completion, or `None` on timeout
        (the goal is kept so the caller can retry). Default timeout 300s."""

    def cancel(self, goal_id: str = None, timeout_s: float = None) -> int:
        """Cancel a goal, or all goals when `goal_id` is `None`.

        Returns the CancelGoalResponse return_code (0=accepted, 1=rejected,
        2=unknown goal, 3=already terminated). Default timeout 10s."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2ActionServer:
    """ROS2 action server. Create via `Ros2Node.create_action_server`.

    warnings:
    - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change.
    - Goals are auto-accepted and driven to Executing on `take_goal`. There is no
    reject path; reject by accepting then `send_result(status="aborted")`.
    - `send_result` blocks until the client requests the result (bounded by a
    send timeout)."""

    def take_goal(
        self, timeout_s: float = None
    ) -> typing.Optional[typing.Tuple[str, pyarrow.Array]]:
        """Wait up to `timeout_s` (default 1s) for the next goal.

        Auto-accepts and starts executing it. Returns `(goal_id, goal_array)`,
        or `None` on timeout. Pass `goal_id` to `send_feedback` / `send_result`."""

    def send_feedback(self, goal_id: str, feedback: pyarrow.Array) -> None:
        """Publish one feedback message for an executing goal."""

    def send_result(
        self,
        goal_id: str,
        result: pyarrow.Array,
        status: str = None,
        timeout_s: float = None,
    ) -> None:
        """Send the terminal result and retire the goal.

        `status` is one of `"succeeded"` (default / `None`), `"aborted"`,
        `"canceled"`; any other value maps to aborted. Blocks until the client
        requests the result, bounded by `timeout_s` (default 300s)."""

    def take_cancel(
        self, timeout_s: float = None
    ) -> typing.Optional[typing.List[str]]:
        """Poll up to `timeout_s` (default 1s) for a cancel request.

        Returns the list of goal_id strings the client asked to cancel that this
        server still tracks (respond to each with `send_result(status="canceled")`),
        or `None` on timeout."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2QosPolicies:
    """ROS2 QoS Policy"""

    def __init__(
        self,
        durability: dora.Ros2Durability = None,
        liveliness: dora.Ros2Liveliness = None,
        reliable: bool = None,
        keep_all: bool = None,
        lease_duration: float = None,
        max_blocking_time: float = None,
        keep_last: int = None,
    ) -> dora.Ros2QosPolicies:
        """ROS2 QoS Policy"""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2Subscription:
    """ROS2 Subscription


    warnings:
    - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change."""

    def next(self): ...
    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

@typing.final
class Ros2Topic:
    """ROS2 Topic

    warnings:
    - Dora ROS2 bridge functionality is considered **unstable**. It may be changed
    at any point without it being considered a breaking change."""

    def __repr__(self) -> str:
        """Return repr(self)."""

    def __str__(self) -> str:
        """Return str(self)."""

def build(
    dataflow_path: str,
    uv: bool = None,
    coordinator_addr: str = None,
    coordinator_port: int = None,
    force_local: bool = False,
) -> None:
    """Build a Dataflow, exactly the same way as `dora build` command line tool."""

def run(dataflow_path: str, uv: bool = None, stop_after: float = None) -> None:
    """Run a Dataflow, exactly the same way as `dora run` command line tool.

    Args:
        dataflow_path: Path to the dataflow YAML file
        uv: Use UV to run Python nodes (optional)
        stop_after: Automatically stop the dataflow after the given duration in seconds (optional)
    """

def start_runtime() -> None:
    """Start a runtime for Operators"""
