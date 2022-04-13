import pickle

from carla import Client, Location, Rotation, Transform, command

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"

client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
world = client.get_world()

town_name = world.get_map().name
dynamic_obstacle_distance_threshold = 1000


def run(inputs):
    keys = inputs.keys()

    if (
        "pose" not in keys
        or "segmented_frame" not in keys
        or "depth_frame" not in keys
    ):
        return {}

    vehicle_transform = pickle.loads(inputs["pose"]).transform

    depth_frame = pickle.loads(inputs["depth_frame"])
    segmented_frame = pickle.loads(inputs["segmented_frame"])

    from pylot.simulation.utils import extract_data_in_pylot_format

    actor_list = world.get_actors()
    (vehicles, people, traffic_lights, _, _) = extract_data_in_pylot_format(
        actor_list
    )

    det_obstacles = []
    for obstacle in vehicles + people:
        # Calculate the distance of the obstacle from the vehicle, and
        # convert to camera view if it is less than
        # dynamic_obstacle_distance_threshold metres away.
        if (
            obstacle.transform.location.distance(vehicle_transform.location)
            <= dynamic_obstacle_distance_threshold
        ):
            bbox = obstacle.populate_bounding_box_2D(
                depth_frame, segmented_frame
            )
            if bbox:
                det_obstacles.append(obstacle)

    return {
        "obstacles_without_location": pickle.dumps(det_obstacles),
        #   "traffic_lights": pickle.dumps(visible_tls),
    }
