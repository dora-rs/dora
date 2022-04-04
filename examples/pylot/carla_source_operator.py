import json
import os
import random
import time

import numpy as np
from carla import Client, Location, Rotation, Transform, command

CARLA_SIMULATOR_HOST = os.environ["CARLA_SIMULATOR_HOST"]
CARLA_SIMULATOR_PORT = os.environ["CARLA_SIMULATOR_PORT"]


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


def spawn_rgb_camera(world, location, rotation, vehicle):
    camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", "1280")
    camera_bp.set_attribute("image_size_y", "720")
    transform = Transform(location=location, rotation=rotation)
    return world.spawn_actor(camera_bp, transform, attach_to=vehicle)


def send_image(image=None):
    return {"image": bytes(msg.raw_data)}


client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
world = client.get_world()

# Spawn the vehicle.
vehicle = spawn_driving_vehicle(client, world)
assert vehicle is not None, "Vehicle is None"
# Spawn the camera and register a function to listen to the images.
camera = spawn_rgb_camera(
    world,
    Location(x=2.0, y=0.0, z=1.8),
    Rotation(roll=0, pitch=0, yaw=0),
    vehicle,
)

camera.listen(send_image)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    # Destroy the actors.
    vehicle.destroy()
    camera.destroy()
