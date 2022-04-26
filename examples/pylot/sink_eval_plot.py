import logging
import threading
import time
from collections import deque

import cv2
import numpy as np
import pygame

import pylot.perception.detection.utils
from dora_watermark import dump, load
from pylot.map.hd_map import HDMap
from pylot.planning.world import World
from pylot.simulation.utils import get_map

mutex = threading.Lock()
pygame.init()

goal_location = pylot.utils.Location(234, 59, 39)


class Flags(object):
    pass


FLAGS = Flags()
FLAGS.tracking_num_steps = 10
FLAGS.step_size = 1
FLAGS.max_iterations = 10
FLAGS.end_dist_threshold = 1
FLAGS.obstacle_clearance_rrt = 1
FLAGS.lane_width = 1
FLAGS.planning_type = "waypoints"
FLAGS.target_speed = 10
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
FLAGS.num_waypoints_ahead = 10
FLAGS.obstacle_clearance_hybrid_astar = 10
FLAGS.lane_width_hybrid_astar = 10
FLAGS.radius = 10
FLAGS.car_length = 10
FLAGS.car_width = 10
FLAGS.static_obstacle_distance_threshold = 1000

goal_location = pylot.utils.Location(234, 59, 39)


logger = logging.Logger("")


display_width = 800
display_height = 600

gameDisplay = pygame.display.set_mode(
    (display_width, display_height),
    pygame.HWSURFACE | pygame.DOUBLEBUF,
)
font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 500)
fontScale = 1
fontColor = (255, 255, 255)
thickness = 1
lineType = 2

counter = time.time()

world = World(FLAGS, logger)

hd_map = HDMap(get_map())


def plot(inputs):
    keys = inputs.keys()
    if "image" not in keys:
        return {}

    if "pose" not in keys:
        return {}

    image, timestamps = load(inputs, "image")

    pose, timestamps = load(inputs, "pose")

    if "obstacles" in keys:
        obstacles, timestamps = load(inputs, "obstacles")
    elif "previous_obstacles" in keys:
        obstacles, timestamps = load(inputs, "previous_obstacles")
    else:
        obstacles = []

    if "waypoints" in keys:
        waypoints, timestamps = load(inputs, "waypoints")
    elif "previous_waypoints" in keys:
        waypoints, timestamps = load(inputs, "previous_waypoints")
    else:
        waypoints = None

    if waypoints is not None:
        waypoints.remove_completed(pose.transform.location)
        waypoints.draw_on_frame(image)

    for obstacle_prediction in obstacles:
        obstacle_prediction.draw_trajectory_on_frame(image)

    global mutex
    mutex.acquire()
    image = image.as_numpy_array()
    if len(image) == 800 * 600 * 4:
        resized_image = np.reshape(image, (display_height, display_width, 4))
        resized_image = resized_image[:, :, :3]
        resized_image = np.ascontiguousarray(resized_image, dtype=np.uint8)
    else:
        resized_image = np.reshape(image, (display_height, display_width, 3))
    global counter
    now = time.time()
    cv2.putText(
        resized_image,
        f"Hertz {1 / (now - counter):.2f}",
        bottomLeftCornerOfText,
        font,
        fontScale,
        fontColor,
        thickness,
        lineType,
    )
    data = resized_image[:, :, (2, 1, 0)]
    data = np.rot90(data)

    counter = now
    pygame.surfarray.blit_array(gameDisplay, data)
    pygame.display.flip()
    mutex.release()

    return {
        "previous_obstacles": dump(obstacles, timestamps),
        "previous_waypoints": dump(waypoints, timestamps),
    }
