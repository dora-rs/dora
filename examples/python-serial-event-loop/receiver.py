"""Dora node that prints counters through SerialEventLoop."""

import logging

from serial_event_loop import InputEvent, SerialEventLoop


def handle_counter(event: InputEvent, emit=print):
    """Print the first counter value in an Arrow input event."""
    if len(event.data) == 0:
        logging.error("[python-receiver] empty counter payload")
        return
    emit(f"[python-receiver] received {event.id}={event.data[0].as_py()}")


def main():
    loop = SerialEventLoop("python-receiver")
    loop.register_handler("counter", handle_counter)
    loop.run()


if __name__ == "__main__":
    main()
