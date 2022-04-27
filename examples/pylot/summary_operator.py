import os
import random
import threading
import time
from datetime import datetime

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

from dora_watermark import load

# You can generate an API token from the "API Tokens Tab" in the UI
token = os.getenv("INFLUX_TOKEN")
token = "iit96Hkq0sYco2sHIuFCM5cU4I5srivYQafgbZgoGmG92gReT9Kao3rNH8b3KFlgPskStVvOOaOU5-LZY94dfA=="
org = "shavtao@gmail.com"
bucket = "DORA Test Bucket"
mutex = threading.Lock()

id = random.randint(0, 1000000)

points = []
counter = 0


def write_to_influxdb(points):
    with InfluxDBClient(
        url="https://eu-central-1-1.aws.cloud2.influxdata.com",
        token=token,
        org=org,
    ) as client:
        write_api = client.write_api(write_options=SYNCHRONOUS)
        write_api.write(bucket, org, points)


def run(inputs):
    if "control_status" not in inputs.keys():
        return {}
    global mutex
    global points
    global counter
    mutex.acquire()
    _, timestamps = load(inputs, "control_status")

    current_time = datetime.utcnow()
    previous_timestamp = timestamps[-1][1]
    for timestamp in timestamps:
        if timestamp[0] == "carla_source_operator":
            points.append(
                Point("Dora-Pylot-Test")
                .tag("host", "host1")
                .tag("id", id)
                .field("global_latency", previous_timestamp - timestamp[1])
                .time(current_time, WritePrecision.NS)
            )
        else:
            points.append(
                Point("Dora-Pylot-Test")
                .tag("host", "host1")
                .tag("id", id)
                .field(timestamp[0], timestamp[1] - previous_timestamp)
                .time(current_time, WritePrecision.NS)
            )
        previous_timestamp = timestamp[1]
    counter += 1

    if counter % 500:
        write_to_influxdb(points)

    mutex.release()
    return {}
