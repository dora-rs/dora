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

    def __init__(self, node_id: str = None) -> None:
        """The custom node API lets you integrate `dora` into your application.
        It allows you to retrieve input and send output in any fashion you want.

        Use with:

        ```python
        from dora import Node

        node = Node()
        ```"""

    def dataflow_descriptor(self) -> dict:
        """Returns the full dataflow descriptor that this node is part of.

        This method returns the parsed dataflow YAML file."""

    def dataflow_id(self) -> str:
        """Returns the dataflow id."""

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
    dora Ros2 bridge functionality is considered **unstable**. It may be changed
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
        dora Ros2 bridge functionality is considered **unstable**. It may be changed
        at any point without it being considered a breaking change.

        ```python
        context = Ros2Context()
        ```"""

    def new_node(
        self, name: str, namespace: str, options: dora.Ros2NodeOptions
    ) -> dora.Ros2Node:
        """Create a new ROS2 node

        ```python
        ros2_node = ros2_context.new_node(
        "turtle_teleop",
        "/ros2_demo",
        Ros2NodeOptions(rosout=True),
        )
        ```

        warning::
        dora Ros2 bridge functionality is considered **unstable**. It may be changed
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
    - dora Ros2 bridge functionality is considered **unstable**. It may be changed
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
        - dora Ros2 bridge functionality is considered **unstable**. It may be changed
        at any point without it being considered a breaking change."""

    def create_subscription(
        self, topic: dora.Ros2Topic, qos: dora.Ros2QosPolicies = None
    ) -> dora.Ros2Subscription:
        """Create a ROS2 subscription

        ```python
        pose_reader = ros2_node.create_subscription(turtle_pose_topic)
        ```

        warnings:
        - dora Ros2 bridge functionality is considered **unstable**. It may be changed
        at any point without it being considered a breaking change."""

    def create_topic(
        self, name: str, message_type: str, qos: dora.Ros2QosPolicies
    ) -> dora.Ros2Topic:
        """Create a ROS2 topic to connect to.

        ```python
        turtle_twist_topic = ros2_node.create_topic(
        "/turtle1/cmd_vel", "geometry_msgs/Twist", topic_qos
        )
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
    - dora Ros2 bridge functionality is considered **unstable**. It may be changed
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
    - dora Ros2 bridge functionality is considered **unstable**. It may be changed
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
    - dora Ros2 bridge functionality is considered **unstable**. It may be changed
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

def run(dataflow_path: str, uv: bool = None) -> None:
    """Run a Dataflow, exactly the same way as `dora run` command line tool."""

def start_runtime() -> None:
    """Start a runtime for Operators"""
