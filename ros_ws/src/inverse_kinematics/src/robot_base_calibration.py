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

try:
    fs = cv2.FileStorage(filename, cv2.FileStorage_READ)
    mtx = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("distortion_coefficients").mat()
    
    fs.release()

    if mtx.size == 0 or dist.size == 0:
        print("파라미터를 찾을 수 없습니다.")
        
    print("camera parameters load success")
except Exception as e:
    print("카메라 파라미터 로드 실패")

joint_angles_rad = np.array([
    0,
    np.pi / 2,
    np.pi / 2,
    0,
    np.pi / 2,
    np.pi / 2
])

transformation_matrix = dofbot_chain.forward_kinematics(joint_angles_rad)

# 데이터 저장 리스트
R_base_ee_list = []
T_base_ee_list = []
R_cam_marker_list = []
T_cam_marker_list = []

def mat_to_rtved(matrix):
    R_matrix = matrix[:3, :3]
    T_vector = matrix[:3, 3]

    rvec, _ = cv2.Rodrigues(R_matrix)
    return rvec, T_vector

def get_robot_pose(curr_joint):
    T_ee_base = dofbot_chain.forward_kinematics(curr_joint)

    R_ee_base = T_ee_base[:3, :3]
    T_ee_base_vec = T_ee_base[:3, 3]

    R_base_ee = R_ee_base.T
    T_base_ee_vec = -R_base_ee @ T_ee_base_vec
    rvec_base_ee, _ = cv2.Rodrigues(R_base_ee)
    return rvec_base_ee, T_base_ee_vec.reshape((3, 1))

def get_marker_pose(frame):
    undistorted_frame = cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    aruco_params = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

 
    if ids is not None:
        rvecs_cam_marker, tvecs_cam_marker, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, mtx, dist
        )
        return rvecs_cam_marker[0], tvecs_cam_marker[0], corners, ids
    return None, None, None, None

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("camera no")
    exit()

print("데이터 수집 시작")
print("로봇 이동 후 엔터 입력")

data_count = 0
while data_count < 20:
    ret, frame = cap.read()
    if not ret:
        break

    rvec_cam_marker, tvec_cam_marker, corners, ids = get_marker_pose(frame)

    display_frame = frame.copy()

    if rvec_cam_marker is not None:
        cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
        cv2.putText(display_frame, "READY", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    else:
        cv2.putText(display_frame, "No marker found", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    cv2.imshow("Calibration Capture", display_frame)

    curr_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    key = cv2.waitKey(1) & 0xFF
    if key == 13 and rvec_cam_marker is not None:
        for i in range(1, 6):
            angle = Arm.Arm_serial_servo_read(i)
            angle2 = Arm.Arm_serial_servo_read(i)
            angle3 = Arm.Arm_serial_servo_read(i)
            time.sleep(0.5)
            if angle is not None:
                curr_angles[i] = np.deg2rad(angle)
            elif angle is None and angle2 is not None:
                curr_angles[i] = np.deg2rad(angle2)
            elif angle is None and angle2 is None and angle3 is not None:
                curr_angles[i] = np.deg2rad(angle3)
        curr_angles = np.array(curr_angles)
        print(curr_angles)

        rvec_base_ee, tvec_base_ee = get_robot_pose(curr_angles)

        R_base_ee_list.append(rvec_base_ee)
        T_base_ee_list.append(tvec_base_ee)
        R_cam_marker_list.append(rvec_cam_marker)
        T_cam_marker_list.append(tvec_cam_marker)

        data_count += 1
        print("Data Captured {} / 20".format(data_count))
        time.sleep(0.5)
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("데이터 수집 완료")

if data_count > 5:
    R_base_ee = np.array(R_base_ee_list)
    T_base_ee = np.array(T_base_ee_list)
    R_cam_marker = np.array(R_cam_marker_list)
    T_cam_marker = np.array(T_cam_marker_list)

    R_cam_base, T_cam_base = cv2.calibrateHandEye(
        R_base_ee,
        T_base_ee,
        R_cam_marker,
        T_cam_marker,
        None,
        None,
        cv2.CALIB_HAND_EYE_TSAI
    )

print("calib result")

T_BASE_CAM = np.eye(4)
T_BASE_CAM[:3, :3] = R_cam_base
T_BASE_CAM[:3, 3] = T_cam_base.flatten()

print("result")
print(T_BASE_CAM)

