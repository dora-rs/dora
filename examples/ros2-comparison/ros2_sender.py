"""ROS2 benchmark sender.

Publishes byte arrays at 10 payload sizes for latency and throughput measurement.
Uses time.perf_counter_ns() for cross-framework comparable timestamps.

Requires: rclpy, std_msgs
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, MultiArrayDimension

LATENCY_SAMPLES = 100
THROUGHPUT_MESSAGES = 100
LATENCY_SLEEP_S = 0.005
SIZES = [0, 8, 64, 512, 2048, 4096, 4 * 4096, 10 * 4096, 100 * 4096, 1000 * 4096]


class BenchSender(Node):
    def __init__(self):
        super().__init__("bench_sender")
        self.latency_pub = self.create_publisher(ByteMultiArray, "bench_latency", 10)
        self.throughput_pub = self.create_publisher(ByteMultiArray, "bench_throughput", 1000)

    def make_msg(self, size: int) -> ByteMultiArray:
        msg = ByteMultiArray()
        msg.data = bytes(size)
        dim = MultiArrayDimension()
        dim.label = "data"
        dim.size = size
        dim.stride = size
        msg.layout.dim.append(dim)
        # Encode send timestamp in layout.data_offset (fits u32 ms resolution)
        # For nanosecond precision, we encode in the first 8 bytes of data
        return msg

    def run(self):
        # Wait for subscriber to connect
        time.sleep(2)

        # Latency test
        for size in SIZES:
            for _ in range(LATENCY_SAMPLES):
                msg = self.make_msg(size)
                # Encode perf_counter_ns in first 8 bytes if payload is large enough
                t = time.perf_counter_ns()
                t_bytes = t.to_bytes(8, "little")
                if size >= 8:
                    msg.data = t_bytes + bytes(size - 8)
                else:
                    msg.data = t_bytes[:size] if size > 0 else b""
                self.latency_pub.publish(msg)
                time.sleep(LATENCY_SLEEP_S)

        time.sleep(2)

        # Throughput test
        for size in SIZES:
            for _ in range(THROUGHPUT_MESSAGES):
                msg = self.make_msg(size)
                t = time.perf_counter_ns()
                t_bytes = t.to_bytes(8, "little")
                if size >= 8:
                    msg.data = t_bytes + bytes(size - 8)
                else:
                    msg.data = t_bytes[:size] if size > 0 else b""
                self.throughput_pub.publish(msg)

            # Sentinel
            sentinel = ByteMultiArray()
            sentinel.data = bytes([1])
            self.throughput_pub.publish(sentinel)
            time.sleep(2)


def main():
    rclpy.init()
    node = BenchSender()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
