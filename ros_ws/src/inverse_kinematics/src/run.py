import cv2, time
from Arm_Lib import Arm_Device
import numpy as np
import datetime

# 변수 초기설정
Arm = Arm_Device()

camera_params_filename = "/home/jetson/camera_params.xml"

initial_pose = [90, 90, 0, 90, 90, 0]
red_pose = [90, 45, 45, 0, 90, 0]
green_pose = [67, 18, 77, 12, 70, 0]
blue_pose = [113, 11, 90, 4, 91, 0]

def arm_grip():
    Arm.Arm_serial_servo_write(6, 180, 1000)
    time.sleep(1)

def arm_release():
    Arm.Arm_serial_servo_write(6, 0, 1000)
    time.sleep(1)

# 마커 크기 설정 (미터 단위)
marker_size = 0.025

def load_camera_parameters(filename):
    try:
        fs = cv2.FileStorage(filename, cv2.FileStorage_READ)
        mtx = fs.getNode("camera_matrix").mat()
        dist = fs.getNode("distortion_coefficients").mat()
        
        fs.release()

        if mtx.size == 0 or dist.size == 0:
            print("파라미터를 찾을 수 없습니다.")
            return None, None
        
        print("camera parameters load success")
        return mtx, dist
    except Exception as e:
        print("카메라 파라미터 로드 실패")
        return None, None
    
    
if __name__ == "__main__":
    # 로봇 대기상태 설정
    Arm.Arm_serial_set_torque(1)
    Arm.Arm_serial_servo_write6_array(initial_pose, 1500)

    # 카메라 파라미터 불러오기
    mtx, dist = load_camera_parameters(camera_params_filename)
    # ArUco 검출기 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    aruco_params = cv2.aruco.DetectorParameters_create()
    print("aruco 검출기 설정 완료")

    # 카메라 초기화
    cap = cv2.VideoCapture(0)
    print("Camera index 0 is On")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # 카메라 초기화 대기
    time.sleep(2)
    # 카메라 인덱스 검사
    ret, frame = cap.read()
    if not ret:
        print("Failed to grap frame on camera 0")
        cap = cv2.VideoCapture(1)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret,frame = cap.read()
        if not ret:
            print("Failed to grap frame on camera 1")
            print("Machine Off")
            Arm.Arm_serial_set_torque(0)
            cap.release()
            exit()
    
    print("Start Camera Loop")
    while True:
        ret, frame = cap.read()
        #print("프레임 받음")
        if not ret:
            print("Failed to grab frame")
            break

        und_frame = cv2.undistort(frame, mtx, dist)

        # Aruco 마커 검출
        corners, ids, rejected = cv2.aruco.detectMarkers(und_frame, aruco_dict, parameters=aruco_params)

        if ids is not None:
            print(ids)
            time.sleep(0.1)

        if ids == 3:
            now = datetime.datetime.now()
            now = now.strftime("%Y-%m-%d %H:%M:%S")
            print("carry start on : {}".format(now))
            Arm.Arm_serial_servo_write6_array(red_pose, 1500)
            time.sleep(1.5)
            arm_grip()
            Arm.Arm_serial_servo_write6(90, 90, 0, 90, 90, 180, 1500)
            time.sleep(1.5)
            Arm.Arm_serial_servo_write(1, 0, 1500)
            time.sleep(1.5)
            arm_release()
            Arm.Arm_serial_servo_write6_array(initial_pose, 1500)
            time.sleep(1.5)
            print("carry complete")
            continue
        elif ids == 2:
            print("종료 신호 수신")
            break

        # cv2.imshow("camera", frame)
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord('q'):
        #     break


cap.release()
cv2.destroyAllWindows()
Arm.Arm_serial_set_torque(0)
print("Program out")