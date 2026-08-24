"""Task-responsive serial event loop helpers for Dora Python nodes."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta
import logging
from queue import Empty, Queue
import threading
import time
from typing import Any, Callable, Mapping


@dataclass(frozen=True)
class InputEvent:
    """An input dispatched by Dora to a registered serial worker."""

    id: str
    data: Any
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class OutputMessage:
    """An output waiting to be sent by the Dora event-loop thread."""

    output_id: str
    data: Any
    metadata: Mapping[str, Any] | None = None


_STOP_WORKER = object()


class SerialWorker:
    """Process input events in FIFO order on a dedicated thread."""

    def __init__(self, name: str, handler: Callable[[InputEvent], None]):
        if not callable(handler):
            raise TypeError("handler must be callable")
        self.name = name
        self._handler = handler
        self._queue: Queue[InputEvent | object] = Queue()
        self._state_lock = threading.Lock()
        self._stopped = False
        self._thread = threading.Thread(
            target=self._process_loop,
            name=f"serial-worker-{name}",
            daemon=False,
        )
        self._thread.start()

    def enqueue(self, event: InputEvent) -> None:
        """Enqueue an event without waiting for its handler to finish."""
        with self._state_lock:
            if self._stopped:
                raise RuntimeError(f"SerialWorker {self.name!r} is stopped")
            self._queue.put(event)

    def stop(self) -> None:
        """Drain accepted events and stop the worker. Safe to call repeatedly."""
        with self._state_lock:
            if not self._stopped:
                self._stopped = True
                self._queue.put(_STOP_WORKER)
        if threading.current_thread() is not self._thread:
            self._thread.join()

    def _process_loop(self) -> None:
        while True:
            event = self._queue.get()
            try:
                if event is _STOP_WORKER:
                    return
                try:
                    self._handler(event)
                except Exception:
                    logging.exception("[%s] input handler failed", self.name)
            finally:
                self._queue.task_done()


@dataclass
class _TimerEvent:
    id: str
    interval_seconds: float
    deadline: float
    handler: Callable[[], None]
    repeat: bool


def _interval_seconds(interval: float | timedelta) -> float:
    seconds = interval.total_seconds() if isinstance(interval, timedelta) else float(interval)
    if seconds <= 0:
        raise ValueError("timer interval must be greater than zero")
    return seconds


class SerialEventLoop:
    """Dispatch Dora inputs to serial workers and manage timer callbacks."""

    def __init__(self, node_name: str, node_factory: Callable[[], Any] | None = None):
        self.node_name = node_name
        self._node_factory = node_factory
        self._workers: dict[str, SerialWorker] = {}
        self._workers_lock = threading.Lock()
        self._timer_condition = threading.Condition()
        self._timers: dict[str, _TimerEvent] = {}
        self._output_queue: Queue[OutputMessage] = Queue()
        self._run_lock = threading.Lock()
        self._has_run = False
        self._running = False
        self._node: Any | None = None
        self._main_thread_id: int | None = None
        self._closing = False
        self._timer_thread = threading.Thread(
            target=self._timer_loop,
            name=f"serial-timers-{node_name}",
            daemon=False,
        )
        self._timer_thread.start()

    def register_handler(
        self, id: str, handler: Callable[[InputEvent], None]
    ) -> SerialWorker:
        """Register or replace the dedicated worker for an input ID."""
        replacement = SerialWorker(id, handler)
        with self._workers_lock:
            if self._closing:
                replacement.stop()
                raise RuntimeError("SerialEventLoop is closed")
            previous = self._workers.get(id)
            self._workers[id] = replacement
        if previous is not None:
            previous.stop()
        return replacement

    def register_timer(
        self,
        id: str,
        interval: float | timedelta,
        handler: Callable[[], None],
        repeat: bool = True,
    ) -> None:
        """Register or replace a timer."""
        if not callable(handler):
            raise TypeError("handler must be callable")
        seconds = _interval_seconds(interval)
        with self._timer_condition:
            if self._closing:
                raise RuntimeError("SerialEventLoop is closed")
            self._timers[id] = _TimerEvent(
                id=id,
                interval_seconds=seconds,
                deadline=time.monotonic() + seconds,
                handler=handler,
                repeat=bool(repeat),
            )
            self._timer_condition.notify_all()

    def cancel_timer(self, id: str) -> bool:
        """Cancel a timer, returning whether it existed."""
        with self._timer_condition:
            removed = self._timers.pop(id, None) is not None
            if removed:
                self._timer_condition.notify_all()
            return removed

    def send_output(
        self,
        output_id: str,
        data: Any,
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        """Queue an output for thread-safe delivery by the Dora loop."""
        message = OutputMessage(output_id, data, metadata)
        if threading.get_ident() == self._main_thread_id and self._node is not None:
            self._send_message(message)
        else:
            self._output_queue.put(message)

    def run(self) -> None:
        """Run the blocking Dora event loop once."""
        with self._run_lock:
            if self._has_run:
                raise RuntimeError("SerialEventLoop.run() can only be called once")
            self._has_run = True
            self._running = True

        self._main_thread_id = threading.get_ident()
        try:
            factory = self._node_factory or _default_node_factory
            self._node = factory()
            while self._running:
                self._drain_outputs()
                event = self._node.next(timeout=0.01)
                self._drain_outputs()

                if event is None:
                    continue
                event_type = event.get("type")
                if event_type == "ERROR":
                    error = str(event.get("error", "unknown Dora error"))
                    if error.startswith("Timeout event stream error:"):
                        continue
                    logging.error("[%s] Dora error: %s", self.node_name, error)
                    continue
                if event_type == "STOP":
                    break
                if event_type == "INPUT":
                    worker = self._get_worker(event.get("id"))
                    if worker is not None:
                        worker.enqueue(
                            InputEvent(
                                id=event["id"],
                                data=event.get("value"),
                                metadata=event.get("metadata") or {},
                            )
                        )
                elif event_type != "TICK":
                    logging.warning(
                        "[%s] unknown Dora event type: %r",
                        self.node_name,
                        event_type,
                    )
        finally:
            self._running = False
            self._stop_timers()
            self._stop_workers()
            if self._node is not None:
                self._drain_outputs()
            self._node = None
            self._main_thread_id = None

    def close(self) -> None:
        """Stop timers and drain all registered workers."""
        self._running = False
        self._stop_timers()
        self._stop_workers()

    def _get_worker(self, id: str | None) -> SerialWorker | None:
        with self._workers_lock:
            return self._workers.get(id) if id is not None else None

    def _send_message(self, message: OutputMessage) -> None:
        try:
            if message.metadata is None:
                self._node.send_output(message.output_id, message.data)
            else:
                self._node.send_output(
                    message.output_id, message.data, message.metadata
                )
        except Exception:
            logging.exception(
                "[%s] failed to send output %r", self.node_name, message.output_id
            )

    def _drain_outputs(self) -> None:
        while True:
            try:
                message = self._output_queue.get_nowait()
            except Empty:
                return
            try:
                self._send_message(message)
            finally:
                self._output_queue.task_done()

    def _stop_timers(self) -> None:
        with self._timer_condition:
            if not self._closing:
                self._closing = True
                self._timers.clear()
                self._timer_condition.notify_all()
        if threading.current_thread() is not self._timer_thread:
            self._timer_thread.join()

    def _stop_workers(self) -> None:
        with self._workers_lock:
            workers = list(self._workers.values())
            self._workers.clear()
        for worker in workers:
            worker.stop()

    def _timer_loop(self) -> None:
        while True:
            with self._timer_condition:
                if self._closing:
                    return
                if not self._timers:
                    self._timer_condition.wait()
                    continue

                timer = min(self._timers.values(), key=lambda item: item.deadline)
                delay = timer.deadline - time.monotonic()
                if delay > 0:
                    self._timer_condition.wait(delay)
                    continue

                current = self._timers.get(timer.id)
                if current is not timer:
                    continue
                if timer.repeat:
                    timer.deadline = time.monotonic() + timer.interval_seconds
                else:
                    del self._timers[timer.id]

            try:
                timer.handler()
            except Exception:
                logging.exception("[%s] timer %r failed", self.node_name, timer.id)


def _default_node_factory() -> Any:
    from dora import Node

    return Node()
