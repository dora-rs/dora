from enum import Enum

from .dora import *


class DoraStatus(Enum):
    """Dora status to indicate if operator `on_input` loop
     should be stopped.

    Args:
        Enum (u8): Status signaling to dora operator to
        stop or continue the operator.
    """

    CONTINUE = 0
    STOP = 1
    STOP_ALL = 2
