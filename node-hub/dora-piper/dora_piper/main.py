from piper_sdk import C_PiperInterface
from dora import Node
import pyarrow as pa
import os
import time

TEACH_MODE = False


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
        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)
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
        print("If you have this issue, you should probably restart your computer")
        exit(0)


def main():
    elapsed_time = time.time()
    CAN_BUS = os.getenv("CAN_BUS", "")
    piper = C_PiperInterface(CAN_BUS)
    piper.ConnectPort()

    if TEACH_MODE is False:
        # piper.MotionCtrl_3(0, 0, 0, 0x00)#位置速度模式
        piper.EnableArm(7)
        enable_fun(piper=piper)
        piper.GripperCtrl(0, 1000, 0x01, 0)
    factor = 57324.840764  # 1000*180/3.14
    time.sleep(2)
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] != "action":
                joint = piper.GetArmJointMsgs()
                gripper = piper.GetArmGripperMsgs()

                joint_value = []
                joint_value += [joint.joint_state.joint_1.real / factor]
                joint_value += [joint.joint_state.joint_2.real / factor]
                joint_value += [joint.joint_state.joint_3.real / factor]
                joint_value += [joint.joint_state.joint_4.real / factor]
                joint_value += [joint.joint_state.joint_5.real / factor]
                joint_value += [joint.joint_state.joint_6.real / factor]
                joint_value += [gripper.gripper_state.grippers_angle / 1000 / 1000 / 4]

                node.send_output("jointstate", pa.array(joint_value, type=pa.float32()))
            else:

                # Do not push to many commands to fast. Limiting it to 20Hz
                # This is due to writing on a moving arm might fail the can bus.
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
                joint_6 = round(position[6] * 1000 * 1000 * 12)

                piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
                piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
                piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
                piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        elif event["type"] == "STOP":

            # Waiting for the arm to stop moving before stopping the node
            time.sleep(5)
            break


if __name__ == "__main__":
    main()
