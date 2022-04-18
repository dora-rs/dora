import pickle

from carla import Client, VehicleControl, command

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))


def run(inputs):
    if "control" not in inputs.keys() or "vehicle_id" not in inputs.keys():
        return {}

    control = pickle.loads(inputs["control"])
    vehicle_id = pickle.loads(inputs["vehicle_id"])

    vec_control = VehicleControl(
        throttle=control["throttle"],
        steer=control["steer"],
        brake=control["brake"],
        hand_brake=False,
        reverse=False,
    )
    client.apply_batch_sync(
        [command.ApplyVehicleControl(vehicle_id, vec_control)]
    )
    return {}
