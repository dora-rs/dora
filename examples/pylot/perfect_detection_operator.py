import time

from carla import Client

from dora_watermark import dump, load

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"

client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
world = client.get_world()

town_name = world.get_map().name
DYNAMIC_OBSTACLE_DISTANCE_THRESHOLD = 500


def run(inputs):
    keys = inputs.keys()

    if (
        "pose" not in keys
        or "segmented_frame" not in keys
        or "depth_frame" not in keys
    ):
        return {}

    pose, timestamps = load(inputs, "pose")
    timestamps.append(("perfect_detection_operator_recieving", time.time()))
    vehicle_transform = pose.transform

    depth_frame, _ = load(inputs, "depth_frame")
    segmented_frame, _ = load(inputs, "segmented_frame")

    from pylot.simulation.utils import extract_data_in_pylot_format

    actor_list = world.get_actors()
    (vehicles, people, _traffic_lights, _, _) = extract_data_in_pylot_format(
        actor_list
    )

    det_obstacles = []
    for obstacle in vehicles + people:
        # Calculate the distance of the obstacle from the vehicle, and
        # convert to camera view if it is less than
        # dynamic_obstacle_distance_threshold metres away.
        if (
            obstacle.transform.location.distance(vehicle_transform.location)
            <= DYNAMIC_OBSTACLE_DISTANCE_THRESHOLD
        ):
            bbox = obstacle.populate_bounding_box_2D(
                depth_frame, segmented_frame
            )
            if bbox:
                det_obstacles.append(obstacle)

    if len(det_obstacles) == 0:
        return {}

    timestamps.append(("perfect_detection_operator", time.time()))

    return {
        "obstacles_without_location": dump(det_obstacles, timestamps),
        #   "traffic_lights": dump(visible_tls),
    }
