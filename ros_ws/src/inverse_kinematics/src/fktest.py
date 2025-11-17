import ikpy.chain
import numpy as np
import cv2, time
from scipy.spatial.transform import Rotation as R
from Arm_Lib import Arm_Device

path = '/home/jetson/project/dofbot_ws/src/dofbot_info/urdf/dofbot.urdf'

filename = "/home/jetson/camera_params.xml"

dofbot_chain = ikpy.chain.Chain.from_urdf_file(path)

DE2RA = np.pi / 180

marker_size = 0.025

Arm = Arm_Device()
joint_angles = []

for i in range(5):
    angle = Arm.Arm_serial_servo_read(i+1)
    joint_angles.append(np.deg2rad(angle))

print(joint_angles)

joint_angles_rad = np.array([
    0,
    np.pi / 2,
    np.pi / 2,
    0,
    np.pi / 2,
    np.pi / 2
])

transformation_matrix = dofbot_chain.forward_kinematics(joint_angles_rad)

print(transformation_matrix)
x = transformation_matrix[0, 3]
y = transformation_matrix[1, 3]
z = transformation_matrix[2, 3]

print("X: {:.4f}, Y: {:.4f}, Z:{:.4f}".format(x, y, z))