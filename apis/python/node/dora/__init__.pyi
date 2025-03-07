import typing

import pyarrow as pa

import dora

@typing.final
class Enum:


    __members__: mappingproxy = ...

@typing.final
class Node:


    def __init__(self, node_id: str | None=None) -> None:
        ...

    def dataflow_descriptor(self) -> dict:
        ...

    def dataflow_id(self) -> str:
        ...

    def merge_external_events(self, subscription: dora.Ros2Subscription) -> None:
        ...

    def next(self, timeout: float | None=None) -> dict:
        ...

    def send_output(self, output_id: str, data: pa.Array, metadata: dict | None=None) -> None:
        ...

    def __iter__(self) -> typing.Any:
        ...

    def __next__(self) -> typing.Any:
        ...

@typing.final
class Ros2Context:


    def __init__(self, ros_paths: list[str] | None=None) -> None:
        ...

    def new_node(self, name: str, namespace: str, options: dora.Ros2NodeOptions) -> dora.Ros2Node:
        ...

@typing.final
class Ros2Durability:


    def __eq__(self, value: object) -> bool:
        ...

    def __ge__(self, value: typing.Any) -> bool:
        ...

    def __gt__(self, value: typing.Any) -> bool:
        ...

    def __int__(self) -> None:
        ...

    def __le__(self, value: typing.Any) -> bool:
        ...

    def __lt__(self, value: typing.Any) -> bool:
        ...

    def __ne__(self, value: object) -> bool:
        ...

    Persistent: Ros2Durability = ...
    Transient: Ros2Durability = ...
    TransientLocal: Ros2Durability = ...
    Volatile: Ros2Durability = ...

@typing.final
class Ros2Liveliness:


    def __eq__(self, value: object) -> bool:
        ...

    def __ge__(self, value: typing.Any) -> bool:
        ...

    def __gt__(self, value: typing.Any) -> bool:
        ...

    def __int__(self) -> None:
        ...

    def __le__(self, value: typing.Any) -> bool:
        ...

    def __lt__(self, value: typing.Any) -> bool:
        ...

    def __ne__(self, value: object) -> bool:
        ...

    Automatic: Ros2Liveliness = ...
    ManualByParticipant: Ros2Liveliness = ...
    ManualByTopic: Ros2Liveliness = ...

@typing.final
class Ros2Node:


    def create_publisher(self, topic: dora.Ros2Topic, qos: dora.Ros2QosPolicies=None) -> dora.Ros2Publisher:
        ...

    def create_subscription(self, topic: dora.Ros2Topic, qos: dora.Ros2QosPolicies=None) -> dora.Ros2Subscription:
        ...

    def create_topic(self, name: str, message_type: str, qos: dora.Ros2QosPolicies) -> dora.Ros2Topic:
        ...

@typing.final
class Ros2NodeOptions:


    def __init__(self, rosout: bool | None=None) -> None:
        ...

@typing.final
class Ros2Publisher:


    def publish(self, data: pa.Array) -> None:
        ...

@typing.final
class Ros2QosPolicies:


    def __init__(self, durability: dora.Ros2Durability=None, liveliness: dora.Ros2Liveliness=None, reliable: bool | None=None, keep_all: bool | None=None, lease_duration: float | None=None, max_blocking_time: float | None=None, keep_last: int | None=None) -> dora.Ros2QoSPolicies:
        ...

@typing.final
class Ros2Subscription:


    def next(self):...

@typing.final
class Ros2Topic:
    ...

def start_runtime() -> None:
    ...
