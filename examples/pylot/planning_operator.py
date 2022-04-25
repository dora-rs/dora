"""
Author: Edward Fang
Email: edward.fang@berkeley.edu
"""
import logging
import threading
import time
from collections import deque

import pylot.utils
from dora_watermark import dump, load
from pylot.map.hd_map import HDMap
from pylot.planning.world import World
from pylot.simulation.utils import get_map

mutex = threading.Lock()


class Flags(object):
    pass


FLAGS = Flags()
FLAGS.tracking_num_steps = 10
FLAGS.planning_type = "rrt"
FLAGS.max_speed = 10.0
FLAGS.max_accel = 6.0
FLAGS.max_curvature = 1.0

# Hybrid AStar flags
FLAGS.step_size_hybrid_astar = 3.0
FLAGS.max_iterations_hybrid_astar = 2000
FLAGS.completion_threshold = 1.0
FLAGS.angle_completion_threshold = 100
FLAGS.rad_step = 4.0
FLAGS.rad_upper_range = 4.0
FLAGS.rad_lower_range = 4.0
FLAGS.obstacle_clearance_hybrid_astar = 1.0
FLAGS.lane_width_hybrid_astar = 6.0
FLAGS.radius = 6.0
FLAGS.car_length = 4.0
FLAGS.car_width = 1.8

FLAGS.dynamic_obstacle_distance_threshold = 50

# Planning general
FLAGS.target_speed = 10.0
FLAGS.obstacle_radius = 1.0
FLAGS.num_waypoints_ahead = 30
FLAGS.num_waypoints_behind = 0
FLAGS.obstacle_filtering_distance = 1.0

# RRT Star
FLAGS.step_size = 0.5
FLAGS.max_iterations = 200
FLAGS.end_dist_threshold = 2.0
FLAGS.obstacle_clearance_rrt = 0.5
FLAGS.lane_width = 3.0

# FOT
FLAGS.max_speed = 35.0  # Maximum vehicle speed [m/s]
FLAGS.max_accel = 6.0  # Maximum vehicle acceleration [m/s^2]
FLAGS.max_curvature = 1.0  # Maximum curvature speed [1/m]
FLAGS.max_road_width_l = 5.0  # Maximum left road width [m]
FLAGS.max_road_width_r = 1.0  # Maximum right road width [m]
FLAGS.d_road_w = 0.25  # Road width sampling discretization [m]
FLAGS.dt = 0.25  # Time sampling discretization [s]
FLAGS.maxt = 8.0  # Max prediction horizon [s]
FLAGS.mint = 2.0  # Min prediction horizon [s]
FLAGS.d_t_s = 0.25  # Target speed sampling discretization [m/s]
FLAGS.n_s_sample = 2.0  # Number speeds to sample
FLAGS.obstacle_clearance_fot = 0.5  # 'Obstacle clearance threshold [m]'
FLAGS.kd = 1.0  # Deviation cost
FLAGS.kv = 0.1  # Velocity cost
FLAGS.ka = 0.1  # Acceleration cost
FLAGS.kj = 0.01  # Jerk cost
FLAGS.kt = 0.01  # Time cost
FLAGS.ko = 0.1  # Obstacle cost
FLAGS.klat = 1.0  # Lateral cost
FLAGS.klon = 1.0  # Longitudinal cost

goal_location = pylot.utils.Location(234, 59, 39)

logger = logging.getLogger("")
logger.setLevel(logging.INFO)
# define file handler and set formatter
file_handler = logging.FileHandler("logfile.log")
formatter = logging.Formatter(
    "%(asctime)s : %(levelname)s : %(name)s : %(message)s"
)
file_handler.setFormatter(formatter)

# add file handler to logger
logger.addHandler(file_handler)
file_handler = logging.FileHandler("logfile.log")


class PlanningOperator:
    """Planning Operator.
    If the operator is running in challenge mode, then it receives all
    the waypoints from the scenario runner agent (on the global trajectory
    stream). Otherwise, it computes waypoints using the HD Map.
    Args:
        pose_stream (:py:class:`erdos.ReadStream`): Stream on which pose
            info is received.
        prediction_stream (:py:class:`erdos.ReadStream`): Stream on which
            trajectory predictions of dynamic obstacles is received.
        static_obstacles_stream (:py:class:`erdos.ReadStream`): Stream on
            which static obstacles (e.g., traffic lights) are received.
        open_drive_stream (:py:class:`erdos.ReadStream`): Stream on which open
            drive string representations are received. The operator can
            construct HDMaps out of the open drive strings.
        route_stream (:py:class:`erdos.ReadStream`): Stream on the planner
            receives high-level waypoints to follow.
        waypoints_stream (:py:class:`erdos.WriteStream`): Stream on which the
            operator sends waypoints the ego vehicle must follow.
        flags (absl.flags): Object to be used to access absl flags.
    """

    def __init__(
        self,
    ):
        # Use the FOT planner for overtaking.
        from pylot.planning.hybrid_astar.hybrid_astar_planner import (
            HybridAStarPlanner,
        )

        self._flags = FLAGS
        self._logger = logger
        self._world = World(self._flags, self._logger)
        self._world._goal_location = goal_location
        self._map = HDMap(get_map())
        self._planner = HybridAStarPlanner(
            self._world, self._flags, self._logger
        )

    def run(self, pose_msg, obstacles, open_drive_msg=None):

        # if open_drive_msg:
        #    self._map = map_from_opendrive(open_drive_msg)

        # Update the representation of the world.
        self._world.update(
            time.time(),
            pose_msg,
            obstacles,
            [],
            hd_map=self._map,
            lanes=None,
        )

        # Total ttd - time spent up to now
        #  ttd = ttd_msg.data - (time.time() - self._world.pose.localization_time)
        # Total ttd - time spent up to now
        speed_factor = 1

        if self._flags.planning_type == "waypoint":
            target_speed = speed_factor * self._flags.target_speed
            output_wps = self._world.follow_waypoints(target_speed)
        else:
            output_wps = self._planner.run(time.time())

        return output_wps


planning = PlanningOperator()


def run(inputs):
    keys = inputs.keys()
    if "pose" not in keys:  # or "open_drive" not in keys:
        return {}

    pose, timestamps = load(inputs, "pose")
    timestamps.append(("planning_operator_recieving", time.time()))

    if "obstacles" in keys:
        obstacles, timestamps = load(inputs, "obstacles")
    elif "previous_obstacles" in keys:
        obstacles, _ = load(inputs, "previous_obstacles")
    else:
        obstacles = deque()

    # open_drive = inputs,"open_drive"].decode("utf-8")
    global mutex
    global planning
    mutex.acquire()
    waypoints = planning.run(pose, obstacles)  # , open_drive)
    mutex.release()

    timestamps.append(("planning_operator", time.time()))

    return {
        "waypoints": dump(waypoints, timestamps),
        "previous_obstacles": dump(obstacles, timestamps),
    }
