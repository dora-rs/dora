"""
Author: Edward Fang
Email: edward.fang@berkeley.edu
"""
import logging
import pickle
import threading
import time

import pylot.utils
from pylot.map.hd_map import HDMap
from pylot.planning.world import World
from pylot.simulation.utils import get_map

mutex = threading.Lock()


class Flags(object):
    pass


FLAGS = Flags()
FLAGS.tracking_num_steps = 10
FLAGS.planning_type = "waypoint"
FLAGS.max_speed = 10
FLAGS.max_accel = 10
FLAGS.max_step_hybrid_astar = 10
FLAGS.step_size_hybrid_astar = 10
FLAGS.max_iterations_hybrid_astar = 10
FLAGS.completion_threshold = 10
FLAGS.angle_completion_threshold = 10
FLAGS.rad_step = 10
FLAGS.rad_upper_range = 10
FLAGS.rad_lower_range = 10
FLAGS.max_curvature = 10
FLAGS.obstacle_clearance_hybrid_astar = 10
FLAGS.lane_width_hybrid_astar = 10
FLAGS.radius = 10
FLAGS.car_length = 10
FLAGS.car_width = 10
FLAGS.dynamic_obstacle_distance_threshold = 100

# Planning general
FLAGS.target_speed = 10
FLAGS.obstacle_radius = 1
FLAGS.num_waypoints_ahead = 6000
FLAGS.num_waypoints_behind = 3000
FLAGS.obstacle_filtering_distance = 1.0

# RRT Star
FLAGS.step_size = 2
FLAGS.max_iterations = 2000
FLAGS.end_dist_threshold = 2.0
FLAGS.obstacle_clearance_rrt = 0.5
FLAGS.lane_width = 3


goal_location = pylot.utils.Location(234, 59, 39)


logger = logging.Logger("")


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
        from pylot.planning.rrt_star.rrt_star_planner import RRTStarPlanner

        self._flags = FLAGS
        self._logger = logger
        self._world = World(self._flags, self._logger)
        self._world._goal_location = goal_location
        self._planner = RRTStarPlanner(self._world, self._flags, self._logger)
        self._map = HDMap(get_map())

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
        speed_factor_stop = 1
        speed_factor_tl = 1
        global mutex
        mutex.acquire()
        if self._flags.planning_type == "waypoint":
            target_speed = speed_factor * self._flags.target_speed
            output_wps = self._world.follow_waypoints(target_speed)
        else:
            output_wps = self._planner.run(time.time())
            speed_factor = min(speed_factor_stop, speed_factor_tl)
            output_wps.apply_speed_factor(speed_factor)
        mutex.release()

        return output_wps


planning = PlanningOperator()


def run(inputs):
    keys = inputs.keys()
    if "pose" not in keys:  # or "open_drive" not in keys:
        return {}

    pose = inputs["pose"]
    pose = pickle.loads(pose)

    if "obstacles" in keys:
        obstacles = pickle.loads(inputs["obstacles"])
        previous_obstacles = inputs["obstacles"]
    elif "previous_obstacles" in keys:
        obstacles = pickle.loads(inputs["previous_obstacles"])
        previous_obstacles = inputs["previous_obstacles"]
    else:
        obstacles = []
        previous_obstacles = pickle.dumps([])

    # open_drive = inputs["open_drive"].decode("utf-8")
    waypoints = planning.run(pose, obstacles)  # , open_drive)
    print(waypoints.as_numpy_array_2D()[0][:5])
    return {
        "waypoints": pickle.dumps(waypoints),
        "previous_obstacles": previous_obstacles,
    }
