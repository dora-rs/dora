from piper_sdk import C_PiperInterface
from dora import Node
import pyarrow as pa
import numpy as np
import os
import time

TEACH_MODE = os.getenv("TEACH_MODE", "False") in ["True", "true"]


def enable_fun(piper: C_PiperInterface):
    """
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    """
    enable_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    while not (enable_flag):
        elapsed_time = time.time() - start_time
        print("--------------------")
        enable_flag = (
            piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status
            and piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status
            and piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status
            and piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status
            and piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status
            and piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        )
        print("使能状态:", enable_flag)
        print("--------------------")
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print("超时....")
            elapsed_time_flag = True
            enable_flag = True
            break
        time.sleep(1)
    if elapsed_time_flag:
        print("程序自动使能超时,退出程序")
        raise ConnectionError("程序自动使能超时,退出程序")


def main():
    elapsed_time = time.time()
    CAN_BUS = os.getenv("CAN_BUS", "")
    piper = C_PiperInterface(CAN_BUS)
    piper.ConnectPort()

    if not TEACH_MODE:
        # piper.MotionCtrl_3(0, 0, 0, 0x00)#位置速度模式
        piper.EnableArm(7)
        enable_fun(piper=piper)
        piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        piper.JointCtrl(0, 0, 0, 0, 0, 0)
        piper.GripperCtrl(abs(0), 1000, 0x01, 0)
        piper.Geten()
        piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        time.sleep(5)

    factor = 57324.840764  # 1000*180/3.14
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "joint_action":
                if TEACH_MODE:
                    continue
                # Do not push to many commands to fast. Limiting it to 20Hz
                if time.time() - elapsed_time > 0.05:
                    elapsed_time = time.time()
                else:
                    continue

                position = event["value"].to_numpy()
                joint_0 = round(position[0] * factor)
                joint_1 = round(position[1] * factor)
                joint_2 = round(position[2] * factor)
                joint_3 = round(position[3] * factor)
                joint_4 = round(position[4] * factor)
                joint_5 = round(position[5] * factor)
                joint_6 = round(position[6] * 1000 * 100)

                piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
                piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
                piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
                piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)

            elif event["id"] == "eef_action":
                if TEACH_MODE:
                    continue
                # Do not push to many commands to fast. Limiting it to 20Hz
                if time.time() - elapsed_time > 0.05:
                    elapsed_time = time.time()
                else:
                    continue

                position = event["value"].to_numpy()
                piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
                piper.EndPoseCtrl(
                    position[0] * 1000 * 1000,
                    position[1] * 1000 * 1000,
                    position[2] * 1000 * 1000,
                    position[3] * 1000 / (2 * np.pi) * 360,
                    position[4] * 1000 / (2 * np.pi) * 360,
                    position[5] * 1000 / (2 * np.pi) * 360,
                )
                piper.GripperCtrl(abs(position[6] * 1000 * 100), 1000, 0x01, 0)
                piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)

            else:
                joint = piper.GetArmJointMsgs()

                joint_value = []
                joint_value += [joint.joint_state.joint_1.real / factor]
                joint_value += [joint.joint_state.joint_2.real / factor]
                joint_value += [joint.joint_state.joint_3.real / factor]
                joint_value += [joint.joint_state.joint_4.real / factor]
                joint_value += [joint.joint_state.joint_5.real / factor]
                joint_value += [joint.joint_state.joint_6.real / factor]

                gripper = piper.GetArmGripperMsgs()
                joint_value += [gripper.gripper_state.grippers_angle / 1000 / 100]

                node.send_output("jointstate", pa.array(joint_value, type=pa.float32()))

                position = piper.GetArmEndPoseMsgs()
                position_value = []
                position_value += [position.end_pose.X_axis * 0.001 * 0.001]
                position_value += [position.end_pose.Y_axis * 0.001 * 0.001]
                position_value += [position.end_pose.Z_axis * 0.001 * 0.001]
                position_value += [position.end_pose.RX_axis * 0.001 / 360 * 2 * np.pi]
                position_value += [position.end_pose.RY_axis * 0.001 / 360 * 2 * np.pi]
                position_value += [position.end_pose.RZ_axis * 0.001 / 360 * 2 * np.pi]

                node.send_output("pose", pa.array(position_value, type=pa.float32()))
                node.send_output(
                    "gripper",
                    pa.array(
                        [gripper.gripper_state.grippers_angle / 1000 / 100],
                        type=pa.float32(),
                    ),
                )

        elif event["type"] == "STOP":

            if not TEACH_MODE:
                piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
                piper.JointCtrl(0, 0, 0, 0, 0, 0)
                piper.GripperCtrl(abs(0), 1000, 0x01, 0)
                piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
            time.sleep(5)
            break


if __name__ == "__main__":
    main()
