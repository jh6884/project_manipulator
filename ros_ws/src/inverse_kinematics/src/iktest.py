import os
from math import pi
from ikpy.chain import Chain
from Arm_Lib import Arm_Device
import numpy as np
import cv2
import time

path = '/home/jetson/project/dofbot_ws/src/dofbot_info/urdf/dofbot.urdf'
dofbot_chain = Chain.from_urdf_file(path)
Arm = Arm_Device()

init_pose = np.array([
    0,
    np.pi / 2,
    np.pi / 2,
    0,
    np.pi / 2,
    np.pi / 2
])

target_matrix = np.eye(4)
target_matrix[0, 3] = 0.1644
target_matrix[1, 3] = -0.0004
target_matrix[2, 3] = 0.0337

ik_result = dofbot_chain.inverse_kinematics(
    target_matrix, initial_position=init_pose
)

print(ik_result)
print(type(ik_result))

arm_servo_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ik_result_list = ik_result.tolist()
print(ik_result_list)
for i in range(5):
    servo_angle = ik_result_list[i+1] * 180 / pi
    arm_servo_angles[i] = round(servo_angle)

print(arm_servo_angles)
Arm.Arm_serial_servo_write6_array(arm_servo_angles, 3000)
del Arm
print("이동완료")
