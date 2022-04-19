import pickle
import threading
import time

mutex = threading.Lock()

from carla import Client, VehicleControl, command

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))

counter = time.time()


def run(inputs):
    if "control" not in inputs.keys() or "vehicle_id" not in inputs.keys():
        return {}
    global mutex
    mutex.acquire()
    global counter
    now = time.time()
    print(f"{now - counter:.2f}")
    counter = now
    mutex.release()

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
