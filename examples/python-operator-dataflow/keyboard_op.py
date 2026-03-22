"""Keyboard operator for capturing user input in dora dataflow."""

import threading
from collections import deque

import pyarrow as pa
from dora import DoraStatus
from pynput import keyboard
from pynput.keyboard import Key

NODE_TOPIC = ["record", "send", "ask", "change"]


class Operator:
    """Keyboard input operator using the modern class-based API."""

    def __init__(self):
        """Initialize the keyboard operator."""
        self.buffer_text = ""
        self.ctrl = False
        self.submitted_text = []
        self.cursor = 0
        self.pending_events = deque()
        self.lock = threading.Lock()

        # Start keyboard listener in a separate thread
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self.listener.start()

    def _on_press(self, key):
        """Handle key press events (called from listener thread)."""
        with self.lock:
            self.pending_events.append(("press", key))

    def _on_release(self, key):
        """Handle key release events (called from listener thread)."""
        with self.lock:
            self.pending_events.append(("release", key))

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        """Handle dora events and process keyboard input.

        Parameters
        ----------
        dora_event : dict
            Event containing an `id`, `type`, `value`, and `metadata`.
        send_output : Callable
            Function for sending output to the dataflow.

        Returns
        -------
        DoraStatus
            CONTINUE to keep listening, STOP to terminate.

        """
        if dora_event["type"] == "INPUT":
            # Handle recording input from whisper
            if dora_event["id"] == "recording":
                self.buffer_text += dora_event["value"][0].as_py()
                send_output("buffer", pa.array([self.buffer_text]), dora_event["metadata"])
                return DoraStatus.CONTINUE

            # Process pending keyboard events on timer tick
            if dora_event["id"] == "tick":
                self._process_keyboard_events(send_output, dora_event["metadata"])

        return DoraStatus.CONTINUE

    def _process_keyboard_events(self, send_output, metadata):
        """Process all pending keyboard events."""
        with self.lock:
            events_to_process = list(self.pending_events)
            self.pending_events.clear()

        for event_type, key in events_to_process:
            if event_type == "press":
                self._handle_key_press(key, send_output, metadata)
            elif event_type == "release":
                self._handle_key_release(key)

    def _handle_key_press(self, key, send_output, metadata):
        """Handle a key press event."""
        if hasattr(key, "char") and key.char is not None:
            self.cursor = 0
            self.buffer_text += key.char
            send_output("buffer", pa.array([self.buffer_text]), metadata)
        elif key == Key.backspace:
            self.buffer_text = self.buffer_text[:-1]
            send_output("buffer", pa.array([self.buffer_text]), metadata)
        elif key == Key.esc:
            self.buffer_text = ""
            send_output("buffer", pa.array([self.buffer_text]), metadata)
        elif key == Key.enter:
            send_output("submitted", pa.array([self.buffer_text]), metadata)
            first_word = self.buffer_text.split(" ")[0]
            if first_word in NODE_TOPIC:
                send_output(first_word, pa.array([self.buffer_text]), metadata)
            self.submitted_text.append(self.buffer_text)
            self.buffer_text = ""
            send_output("buffer", pa.array([self.buffer_text]), metadata)
        elif key == Key.ctrl:
            self.ctrl = True
        elif key == Key.space:
            self.buffer_text += " "
            send_output("buffer", pa.array([self.buffer_text]), metadata)
        elif key == Key.up:
            if len(self.submitted_text) > 0:
                self.cursor = max(self.cursor - 1, -len(self.submitted_text))
                self.buffer_text = self.submitted_text[self.cursor]
                send_output("buffer", pa.array([self.buffer_text]), metadata)
        elif key == Key.down:
            if len(self.submitted_text) > 0:
                self.cursor = min(self.cursor + 1, 0)
                self.buffer_text = self.submitted_text[self.cursor]
                send_output("buffer", pa.array([self.buffer_text]), metadata)

    def _handle_key_release(self, key):
        """Handle a key release event."""
        if key == Key.ctrl:
            self.ctrl = False

    def __del__(self):
        """Clean up the keyboard listener."""
        if hasattr(self, "listener") and self.listener.is_alive():
            self.listener.stop()
