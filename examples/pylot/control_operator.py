import threading
import time

from dora_watermark import dump, load

mutex = threading.Lock()

from carla import Client, VehicleControl, command

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))

vehicle_id = None


def run(inputs):

    global vehicle_id

    if vehicle_id is None and "vehicle_id" not in inputs.keys():
        return {}
    elif vehicle_id is None and "vehicle_id" in inputs.keys():
        global mutex
        mutex.acquire()
        vehicle_id, _ = load(inputs, "vehicle_id")
        mutex.release()

    if "control" not in inputs.keys():
        return {}

    control, timestamps = load(inputs, "control")
    timestamps.append(("control_operator_recieving", time.time()))

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
    timestamps.append(("control_operator", time.time()))
    return {"control_status": dump(1, timestamps)}
