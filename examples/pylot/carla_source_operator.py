import json
import os
import pickle
import random
import time

import numpy as np
from carla import Client, Location, Rotation, Transform, command

import pylot.simulation.utils
import pylot.utils
from pylot.drivers.sensor_setup import CameraSetup, LidarSetup
from pylot.perception.camera_frame import CameraFrame
from pylot.perception.Depth_frame import DepthFrame

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
LABELS = "image"
last_frame = None
last_depth_frame = None
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600


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
        or not int(vehicle_bp.get_attribute("number_of_wheels")) == 4
    ):
        vehicle_bp = random.choice(
            world.get_blueprint_library().filter("vehicle.*")
        )
    vehicle_bp.set_attribute("role_name", "autopilot")

    # Get the spawn point of the vehicle.
    start_pose = random.choice(world.get_map().get_spawn_points())

    # Spawn the vehicle.
    batch = [
        command.SpawnActor(vehicle_bp, start_pose).then(
            command.SetAutopilot(command.FutureActor, True)
        )
    ]
    vehicle_id = client.apply_batch_sync(batch)[0].actor_id
    while world.get_actors().find(vehicle_id) is None:

        # Find the vehicle and return the Actor instance.
        time.sleep(
            5
        )  # This is so that the vehicle gets registered in the actors.
    return world.get_actors().find(vehicle_id)


def add_camera(world, location, rotation, callback, vehicle):
    camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", f"{IMAGE_WIDTH}")
    camera_bp.set_attribute("image_size_y", f"{IMAGE_HEIGHT}")
    camera_bp.set_attribute("sensor_tick", "0.1")
    transform = Transform(location=location, rotation=rotation)
    camera = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
    camera.listen(callback)
    return camera


def add_lidar(world, location, rotation, callback, vehicle):
    lidar_blueprint = world.get_blueprint_library().find(
        "sensor.lidar.ray_cast"
    )
    lidar_blueprint.set_attribute("channels", "32")
    lidar_blueprint.set_attribute("range", "5000")
    lidar_blueprint.set_attribute("points_per_second", "500000")
    lidar_blueprint.set_attribute("rotation_frequency", "20")
    lidar_blueprint.set_attribute("upper_fov", "15")
    lidar_blueprint.set_attribute("lower_fov", "-30")
    transform = Transform(location=location, rotation=rotation)
    lidar = world.spawn_actor(lidar_blueprint, transform, attach_to=vehicle)
    # Register callback to be invoked when a new point cloud is received.
    lidar.listen(callback)
    return lidar


def on_camera_msg(simulator_image):
    game_time = int(simulator_image.timestamp * 1000)

    camera_transform = pylot.utils.Transform.from_simulator_transform(
        simulator_image.transform
    )

    camera_setup = CameraSetup(
        "rgb_camera",
        "sensor.camera.rgb",
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        camera_transform,
        fov=90.0,
    )
    global last_frame
    last_frame = pickle.dumps(
        CameraFrame.from_simulator_frame(simulator_image, camera_setup)
    )


def on_lidar_msg(simulator_lidar):
    game_time = int(simulator_lidar.timestamp * 1000)

    lidar_transform = pylot.utils.Transform.from_simulator_transform(
        simulator_lidar.transform
    )

    camera_setup = CameraSetup(
        "rgb_camera",
        "sensor.camera.rgb",
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        lidar_transform,
        fov=90.0,
    )
    global last_depth_frame
    last_depth_frame = pickle.dumps(
        DepthFrame.from_simulator_frame(simulator_lidar, camera_setup)
    )


client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
world = client.get_world()

# Spawn the vehicle.
vehicle = spawn_driving_vehicle(client, world)
assert vehicle is not None, "Vehicle is None"

# Spawn the camera and register a function to listen to the images.
camera = add_camera(
    world,
    Location(x=2.0, y=0.0, z=1.8),
    Rotation(roll=0, pitch=0, yaw=0),
    on_camera_msg,
    vehicle,
)

# Spawn lidar camera.
camera = add_lidar(
    world,
    Location(x=2.0, y=0.0, z=1.8),
    Rotation(roll=0, pitch=0, yaw=0),
    on_lidar_msg,
    vehicle,
)


def send(_):
    if last_frame is None:
        return {}

    vec_transform = pylot.utils.Transform.from_simulator_transform(
        vehicle.get_transform()
    )
    velocity_vector = pylot.utils.Vector3D.from_simulator_vector(
        vehicle.get_velocity()
    )
    forward_speed = velocity_vector.magnitude()
    pose = pylot.utils.Pose(vec_transform, forward_speed, velocity_vector)
    binary_data = pickle.dumps(pose)

    return {
        "image": last_frame,
        "pose": binary_data,
        #  "open_drive": world.get_map().to_opendrive().encode("utf-8"),
    }
