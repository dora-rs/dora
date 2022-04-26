import logging
import threading
import time

import pylot.control.utils
import pylot.planning.utils
from dora_watermark import dump, load
from pylot.control.pid import PIDLongitudinalController

mutex = threading.Lock()
MIN_PID_STEER_WAYPOINT_DISTANCE = 5
MIN_PID_SPEED_WAYPOINT_DISTANCE = 5
STEER_GAIN = 0.7
COAST_FACTOR = 1.75
pid_p = 1.0
pid_d = 0.0
pid_i = 0.05
dt = 1.0 / 3
pid_use_real_time = True
pid = PIDLongitudinalController(pid_p, pid_d, pid_i, dt, pid_use_real_time)

logger = logging.Logger("")


class Flags(object):
    pass


FLAGS = Flags()
FLAGS.brake_max = 1.0
FLAGS.throttle_max = 0.5


def run(inputs):

    keys = inputs.keys()
    if "pose" not in keys:
        return {}

    pose, timestamps = load(inputs, "pose")
    timestamps.append(("pid_control_operator_recieving", time.time()))
    ego_transform = pose.transform
    # Vehicle speed in m/s.
    current_speed = pose.forward_speed
    if "waypoints" in keys:
        waypoints, timestamps = load(inputs, "waypoints")
    elif "previous_waypoints" in keys:
        waypoints, _ = load(inputs, "previous_waypoints")
    else:
        return {}

    global mutex
    mutex.acquire()
    waypoints.remove_completed(ego_transform.location)

    try:
        angle_steer = waypoints.get_angle(
            ego_transform, MIN_PID_STEER_WAYPOINT_DISTANCE
        )

        target_speed = waypoints.get_target_speed(
            ego_transform, MIN_PID_SPEED_WAYPOINT_DISTANCE
        )

        throttle, brake = pylot.control.utils.compute_throttle_and_brake(
            pid, current_speed, target_speed, FLAGS, logger
        )

        steer = pylot.control.utils.radians_to_steer(angle_steer, STEER_GAIN)
    except ValueError:
        print("Braking! No more waypoints to follow.")
        throttle, brake = 0.0, 0.5
        steer = 0.0
    mutex.release()

    timestamps.append(("PID_control_operator", time.time()))

    return {
        "control": dump(
            {"steer": steer, "throttle": throttle, "brake": brake}, timestamps
        ),
        "previous_waypoints": dump(waypoints, timestamps),
    }
