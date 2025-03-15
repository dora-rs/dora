from robot import Robot
from dynamixel import Dynamixel

leader_dynamixel = Dynamixel.Config(
    baudrate=1_000_000, device_name="/dev/ttyDXL_master_right"
).instantiate()
follower_dynamixel = Dynamixel.Config(
    baudrate=1_000_000, device_name="/dev/ttyDXL_puppet_right"
).instantiate()
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6, 7, 8])
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6, 7, 8])
# leader.set_trigger_torque()

import time

times_rec = []
times_send = []

while True:
    now = time.time()
    pos = leader.read_position()
    # print(f"Time to rec pos: {(time.time() - now) * 1000}")
    follower.set_goal_pos(pos)
    print(f"Time to send pos: {(time.time() - now) * 1000}")
