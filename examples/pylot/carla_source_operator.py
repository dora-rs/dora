import logging
import pickle
import random
import time

from carla import Client, Location, Rotation, Transform, command

import pylot.simulation.utils
import pylot.utils
from pylot.drivers.sensor_setup import (
    CameraSetup,
    LidarSetup,
    SegmentedCameraSetup,
)
from pylot.perception.camera_frame import CameraFrame
from pylot.perception.depth_frame import DepthFrame
from pylot.perception.point_cloud import PointCloud
from pylot.perception.segmentation.segmented_frame import SegmentedFrame

logger = logging.Logger("")


CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
LABELS = "image"
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600

lidar_pc = None
depth_frame = None
last_frame = None
segmented_frame = None
vehicle_id = None

sensor_transform = Transform(
    Location(3, 0, 1), Rotation(pitch=0, yaw=0, roll=0)
)

goal_location = pylot.utils.Location(234, 59, 39)


def on_segmented_msg(simulator_image):
    transform = pylot.utils.Transform.from_simulator_transform(
        simulator_image.transform
    )
    camera_setup = SegmentedCameraSetup(
        "segmented_camera",
        800,
        600,
        transform,
        90.0,
    )

    global segmented_frame
    segmented_frame = SegmentedFrame.from_simulator_image(
        simulator_image, camera_setup
    )


def on_lidar_msg(simulator_pc):
    lidar_transform = pylot.utils.Transform.from_simulator_transform(
        simulator_pc.transform
    )
    lidar_setup = LidarSetup(
        "lidar", lidar_type="sensor.lidar.ray_cast", transform=lidar_transform
    )

    global lidar_pc
    lidar_pc = PointCloud.from_simulator_point_cloud(simulator_pc, lidar_setup)
    # pptk.viewer(point_cloud.points)


def on_camera_msg(simulator_image):

    camera_transform = pylot.utils.Transform.from_simulator_transform(
        simulator_image.transform
    )

    camera_setup = CameraSetup(
        "rgb_camera", "sensor.camera.rgb", 800, 600, camera_transform, fov=90.0
    )

    global last_frame
    last_frame = CameraFrame.from_simulator_frame(simulator_image, camera_setup)


def on_depth_msg(simulator_image):

    depth_camera_transform = pylot.utils.Transform.from_simulator_transform(
        simulator_image.transform
    )

    camera_setup = CameraSetup(
        "depth_camera",
        "sensor.camera.depth",
        800,
        600,
        depth_camera_transform,
        fov=90.0,
    )
    global depth_frame
    depth_frame = DepthFrame.from_simulator_frame(simulator_image, camera_setup)

    # pptk.viewer(depth_pc)


def add_lidar(world, transform, callback, vehicle):
    lidar_blueprint = world.get_blueprint_library().find(
        "sensor.lidar.ray_cast"
    )
    lidar_blueprint.set_attribute("channels", "32")
    lidar_blueprint.set_attribute("range", "5000")
    lidar_blueprint.set_attribute("points_per_second", "500000")
    lidar_blueprint.set_attribute("rotation_frequency", "20")
    lidar_blueprint.set_attribute("upper_fov", "15")
    lidar_blueprint.set_attribute("lower_fov", "-30")
    lidar = world.spawn_actor(lidar_blueprint, transform, attach_to=vehicle)
    # Register callback to be invoked when a new point cloud is received.
    lidar.listen(callback)
    return lidar


def add_depth_camera(world, transform, callback, vehicle):
    depth_blueprint = world.get_blueprint_library().find("sensor.camera.depth")
    depth_blueprint.set_attribute("image_size_x", "800")
    depth_blueprint.set_attribute("image_size_y", "600")
    depth_camera = world.spawn_actor(
        depth_blueprint, transform, attach_to=vehicle
    )
    # Register callback to be invoked when a new frame is received.
    depth_camera.listen(callback)
    return depth_camera


def add_camera(world, transform, callback, vehicle):
    camera_blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_blueprint.set_attribute("image_size_x", "800")
    camera_blueprint.set_attribute("image_size_y", "600")
    camera = world.spawn_actor(camera_blueprint, transform, attach_to=vehicle)
    # Register callback to be invoked when a new frame is received.
    camera.listen(callback)
    return camera


def add_segmented_camera(world, transform, callback, vehicle):
    segmented_blueprint = world.get_blueprint_library().find(
        "sensor.camera.semantic_segmentation"
    )
    segmented_blueprint.set_attribute("image_size_x", str(IMAGE_WIDTH))
    segmented_blueprint.set_attribute("image_size_y", str(IMAGE_HEIGHT))
    segmented_blueprint.set_attribute("fov", str(90.0))
    segmented_camera = world.spawn_actor(
        segmented_blueprint, transform, attach_to=vehicle
    )
    segmented_camera.listen(callback)
    return segmented_camera


def spawn_driving_vehicle(client, world):
    """This function spawns the driving vehicle and puts it into
    an autopilot mode.
    Args:
        client: The Client instance representing the simulation to
          connect to.
        world: The world inside the current simulation.
    Returns:
        A Actor instance representing the vehicle that was just spawned.
    """
    # Get the blueprint of the vehicle and set it to AutoPilot.
    vehicle_bp = random.choice(
        world.get_blueprint_library().filter("vehicle.*")
    )
    while (
        not vehicle_bp.has_attribute("number_of_wheels")
        or int(vehicle_bp.get_attribute("number_of_wheels")) != 4
    ):
        vehicle_bp = random.choice(
            world.get_blueprint_library().filter("vehicle.*")
        )
    # vehicle_bp.set_attribute("role_name", "autopilot")

    # Get the spawn point of the vehicle.
    start_pose = random.choice(world.get_map().get_spawn_points())

    # Spawn the vehicle.
    batch = [
        command.SpawnActor(vehicle_bp, start_pose).then(
            command.SetAutopilot(command.FutureActor, False)
        )
    ]

    global vehicle_id
    vehicle_id = client.apply_batch_sync(batch)[0].actor_id
    while world.get_actors().find(vehicle_id) is None:

        # Find the vehicle and return the Actor instance.
        time.sleep(
            5
        )  # This is so that the vehicle gets registered in the actors.
        print("waiting for ego vehicle to create")
    return world.get_actors().find(vehicle_id)


client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
world = client.get_world()
# settings = world.get_settings()
# settings.synchronous_mode = True
# settings.fixed_delta_seconds = 1.0 / 10
# world.apply_settings(settings)

# Spawn the vehicle.
(_, vehicle_ids, people) = pylot.simulation.utils.spawn_actors(
    client,
    world,
    8000,
    "0.9.10",
    -1,
    True,
    10,
    10,
    logger,
)

ego_vehicle = spawn_driving_vehicle(client, world)
# lidar = add_lidar(world, sensor_transform, on_lidar_msg, vehicle)
depth_camera = add_depth_camera(
    world, sensor_transform, on_depth_msg, ego_vehicle
)
camera = add_camera(world, sensor_transform, on_camera_msg, ego_vehicle)
segmented_camera = add_segmented_camera(
    world, sensor_transform, on_segmented_msg, ego_vehicle
)


def send(_):
    global last_frame
    global segmented_frame
    global depth_frame

    if last_frame is None or segmented_frame is None or depth_frame is None:
        return {}

    vec_transform = pylot.utils.Transform.from_simulator_transform(
        ego_vehicle.get_transform()
    )
    velocity_vector = pylot.utils.Vector3D.from_simulator_vector(
        ego_vehicle.get_velocity()
    )
    forward_speed = velocity_vector.magnitude()
    pose = pylot.utils.Pose(vec_transform, forward_speed, velocity_vector)

    return {
        "image": pickle.dumps(last_frame),
        "depth_frame": pickle.dumps(depth_frame),
        "segmented_frame": pickle.dumps(segmented_frame),
        "pose": pickle.dumps(pose),
        "vehicle_id": pickle.dumps(vehicle_id)
        #  "open_drive": world.get_map().to_opendrive().encode("utf-8"),
    }
