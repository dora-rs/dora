from datetime import timedelta
from queue import Empty, Queue
import threading
import time
import unittest

from serial_event_loop import InputEvent, SerialEventLoop, SerialWorker


class SerialWorkerTests(unittest.TestCase):
    def test_processes_events_in_fifo_order_and_drains_on_stop(self):
        processed = []
        worker = SerialWorker("camera", lambda event: processed.append(event.data))
        for value in range(20):
            worker.enqueue(InputEvent("camera", value))

        worker.stop()

        self.assertEqual(processed, list(range(20)))

    def test_stop_is_idempotent_and_enqueue_after_stop_is_rejected(self):
        worker = SerialWorker("camera", lambda event: None)
        worker.stop()
        worker.stop()

        with self.assertRaisesRegex(RuntimeError, "stopped"):
            worker.enqueue(InputEvent("camera", 1))

    def test_handler_failure_does_not_discard_later_events(self):
        processed = []

        def handler(event):
            if event.data == "bad":
                raise ValueError("broken input")
            processed.append(event.data)

        worker = SerialWorker("camera", handler)
        worker.enqueue(InputEvent("camera", "bad"))
        worker.enqueue(InputEvent("camera", "good"))
        worker.stop()

        self.assertEqual(processed, ["good"])


class TimerTests(unittest.TestCase):
    def test_rejects_non_positive_intervals(self):
        loop = SerialEventLoop("test")
        for interval in (0, -0.01, timedelta(0)):
            with self.subTest(interval=interval):
                with self.assertRaises(ValueError):
                    loop.register_timer("bad", interval, lambda: None)
        loop.close()

    def test_repeating_timer_fires_and_can_be_cancelled(self):
        loop = SerialEventLoop("test")
        fired = threading.Event()
        calls = []

        def callback():
            calls.append(time.monotonic())
            if len(calls) >= 2:
                fired.set()

        loop.register_timer("tick", 0.01, callback)
        self.assertTrue(fired.wait(0.5))
        self.assertTrue(loop.cancel_timer("tick"))
        self.assertFalse(loop.cancel_timer("tick"))
        count = len(calls)
        time.sleep(0.04)
        loop.close()

        self.assertEqual(len(calls), count)

    def test_one_shot_timedelta_timer_fires_once(self):
        loop = SerialEventLoop("test")
        fired = threading.Event()
        calls = []
        loop.register_timer(
            "once",
            timedelta(milliseconds=10),
            lambda: (calls.append("once"), fired.set()),
            repeat=False,
        )

        self.assertTrue(fired.wait(0.5))
        time.sleep(0.04)
        loop.close()

        self.assertEqual(calls, ["once"])
        self.assertFalse(loop.cancel_timer("once"))

    def test_replacing_handler_stops_the_old_worker(self):
        loop = SerialEventLoop("test")
        first = loop.register_handler("camera", lambda event: None)
        second = loop.register_handler("camera", lambda event: None)
        loop.close()

        with self.assertRaises(RuntimeError):
            first.enqueue(InputEvent("camera", 1))
        with self.assertRaises(RuntimeError):
            second.enqueue(InputEvent("camera", 1))


_END = object()
_TIMEOUT = object()


class FakeNode:
    def __init__(self, events):
        self.events = Queue()
        for event in events:
            self.events.put(event)
        self.events.put(_END)
        self.sent = []
        self.send_threads = []

    def next(self, timeout=None):
        try:
            event = self.events.get(timeout=timeout)
        except Empty:
            return {"type": "TICK"}
        if event is _END:
            return {"type": "STOP"}
        if event is _TIMEOUT:
            return None
        return event

    def send_output(self, output_id, data, metadata=None):
        self.send_threads.append(threading.get_ident())
        self.sent.append((output_id, data, metadata))


class DoraLoopTests(unittest.TestCase):
    def test_timeout_error_is_silent_and_does_not_end_the_event_loop(self):
        received = []
        fake = FakeNode(
            [
                {
                    "type": "ERROR",
                    "error": "Timeout event stream error: Receiver timed out",
                },
                {"type": "INPUT", "id": "camera", "value": "frame"},
                {"type": "STOP"},
            ]
        )
        loop = SerialEventLoop("test", node_factory=lambda: fake)
        loop.register_handler("camera", received.append)

        with self.assertNoLogs(level="WARNING"):
            loop.run()

        self.assertEqual([event.data for event in received], ["frame"])

    def test_real_dora_error_is_logged(self):
        fake = FakeNode(
            [
                {"type": "ERROR", "error": "transport disconnected"},
                {"type": "STOP"},
            ]
        )
        loop = SerialEventLoop("test", node_factory=lambda: fake)

        with self.assertLogs(level="ERROR") as logs:
            loop.run()

        self.assertTrue(
            any("transport disconnected" in message for message in logs.output)
        )

    def test_timeout_does_not_end_the_event_loop(self):
        received = []
        fake = FakeNode(
            [
                _TIMEOUT,
                {"type": "INPUT", "id": "camera", "value": "frame"},
                {"type": "STOP"},
            ]
        )
        loop = SerialEventLoop("test", node_factory=lambda: fake)
        loop.register_handler("camera", received.append)

        loop.run()

        self.assertEqual([event.data for event in received], ["frame"])

    def test_dispatches_arrow_value_and_metadata_to_registered_worker(self):
        arrow_value = object()
        metadata = {"trace": "abc"}
        fake = FakeNode(
            [
                {
                    "type": "INPUT",
                    "id": "camera",
                    "value": arrow_value,
                    "metadata": metadata,
                },
                {"type": "STOP"},
            ]
        )
        received = []
        loop = SerialEventLoop("test", node_factory=lambda: fake)
        loop.register_handler("camera", received.append)

        loop.run()

        self.assertEqual(len(received), 1)
        self.assertEqual(received[0], InputEvent("camera", arrow_value, metadata))

    def test_worker_outputs_are_sent_by_event_loop_thread_in_fifo_order(self):
        class BlockingFakeNode(FakeNode):
            def next(self, timeout=None):
                if len(self.sent) >= 2:
                    return {"type": "STOP"}
                time.sleep(min(timeout or 0.01, 0.01))
                return {"type": "TICK"}

        fake = BlockingFakeNode([])
        loop = SerialEventLoop("test", node_factory=lambda: fake)

        def handler(event):
            loop.send_output("result", event.data, {"seq": event.data})

        worker = loop.register_handler("camera", handler)
        worker.enqueue(InputEvent("camera", 1))
        worker.enqueue(InputEvent("camera", 2))
        run_thread_id = []

        def run_loop():
            run_thread_id.append(threading.get_ident())
            loop.run()

        thread = threading.Thread(target=run_loop)
        thread.start()
        thread.join(1.0)
        self.assertFalse(thread.is_alive())

        self.assertEqual(
            fake.sent,
            [
                ("result", 1, {"seq": 1}),
                ("result", 2, {"seq": 2}),
            ],
        )
        self.assertEqual(fake.send_threads, [run_thread_id[0], run_thread_id[0]])

    def test_run_is_single_use(self):
        loop = SerialEventLoop("test", node_factory=lambda: FakeNode([]))
        loop.run()
        with self.assertRaisesRegex(RuntimeError, "once"):
            loop.run()

    def test_send_failure_does_not_block_later_outputs(self):
        class FailingSendNode(FakeNode):
            def __init__(self):
                super().__init__([])
                self.attempts = []

            def next(self, timeout=None):
                if len(self.attempts) >= 2:
                    return {"type": "STOP"}
                time.sleep(min(timeout or 0.01, 0.01))
                return {"type": "TICK"}

            def send_output(self, output_id, data, metadata=None):
                self.attempts.append(output_id)
                if len(self.attempts) == 1:
                    raise RuntimeError("send failed")

        fake = FailingSendNode()
        loop = SerialEventLoop("test", node_factory=lambda: fake)
        loop.send_output("first", 1)
        loop.send_output("second", 2)

        loop.run()

        self.assertEqual(fake.attempts, ["first", "second"])


class TimerFailureTests(unittest.TestCase):
    def test_timer_failure_does_not_stop_other_timers(self):
        loop = SerialEventLoop("test")
        healthy_fired = threading.Event()

        def failing_callback():
            loop.cancel_timer("failing")
            raise RuntimeError("timer failed")

        loop.register_timer("failing", 0.01, failing_callback)
        loop.register_timer("healthy", 0.02, healthy_fired.set, repeat=False)

        self.assertTrue(healthy_fired.wait(0.5))
        loop.close()


if __name__ == "__main__":
    unittest.main()
