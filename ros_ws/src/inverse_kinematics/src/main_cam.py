#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2, time, rospy
from math import pi
# from ikpy.chain import Chain
from Arm_Lib import Arm_Device
import numpy as np
from ikpy.chain import Chain

# 초기설정
path = '/home/jetson/project/dofbot_ws/src/dofbot_info/urdf/dofbot.urdf'
dofbot_chain = Chain.from_urdf_file(path)
Arm = Arm_Device()
# print(f"현재 opencv 버전: {cv2.__version__}")
camera_params_filename = "/home/jetson/camera_params.xml"
DE2RA = pi / 180

# 팔을 초기위치로 이동
Arm.Arm_serial_servo_write6(90, 90, 0, 90, 90, 0, 2000)
time.sleep(2)

def inverse_kinematics_mov(dofbot_chain, target_position):
    # 모터 각도 조절을 위한 변수 생성
    arm_servo_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(6):
        # 팔의 현재 상태를 받아옴
        
        init_pose = Arm.Arm_serial_servo_read(i+1)
        if init_pose is not None:
            arm_servo_angles[i] = init_pose * DE2RA
            print(arm_servo_angles[i])
            time.sleep(0.5)
        else: arm_servo_angles[i] = pi/2

    initial_position = np.array([
        0,
        arm_servo_angles[0],
        arm_servo_angles[1],
        arm_servo_angles[2],
        arm_servo_angles[3],
        arm_servo_angles[4]
    ])

    # target_matrix = np.eye(4)
    # target_matrix[0, 3] = pos_x
    # target_matrix[1, 3] = pos_y
    # target_matrix[2, 3] = pos_z
    # print(target_matrix[:3, 3])

    # 역기구학 계산
    ik_result = dofbot_chain.inverse_kinematics(
        target_position, initial_position=initial_position
    )

    # 디버깅용 출력
    print(ik_result)

    # 디버깅용 출력
    ik_result_list = ik_result.tolist()
    print(ik_result_list)
 
    for i in range(5):
        arm_servo_angles[i] = round(ik_result_list[i+1] * 180 / pi)

    # 결과 출력, 로봇팔 이동
    print(arm_servo_angles)
    Arm.Arm_serial_servo_write6_array(arm_servo_angles, 2000)

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
    mtx, dist = load_camera_parameters(camera_params_filename)
    # ArUco 검출기 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    aruco_params = cv2.aruco.DetectorParameters_create()
    print("aruco 검출기 설정 성공")
    
    """# 노드 초기화
    rospy.init_node("dofbot_carrier")

    print("dofbot 설정 시작")
    dofbot = MoveGroupCommander("dofbot", robot_description="/robot_description", ns="dofbot", wait_for_servers=300.0)

    dofbot.allow_replanning(True)
    print("dofbot allowed replanning")
    dofbot.set_planning_time(5)
    print("dofbot planning time set by 5")
    dofbot.set_num_planning_attempts(10)
    print("dofbot planning attempts 10")
    dofbot.set_goal_position_tolerance(0.01)
    print("dofbot goal position tolerance 0.01")
    dofbot.set_goal_orientation_tolerance(0.01)
    print("dofbot goal orientation tolerance 0.01")
    dofbot.set_goal_tolerance(0.01)
    print("dofbot set goal tolerance 0.01")
    dofbot.set_max_velocity_scaling_factor(1.0)
    print("dofbot max velocity 1.0")
    dofbot.set_max_acceleration_scaling_factor(1.0)
    print("dofbot max acceleration 1.0")
    dofbot.set_named_target("down")
    print("dofbot set named target down")
    dofbot.go()
    print("dofbot settings complete")
    time.sleep(0.5)"""
    
    # 마커 크기 설정 (미터 단위)
    marker_size = 0.025  # 예: 5cm = 0.05m
    
    # 카메라 설정
    print("카메라 켜기")
    cap = cv2.VideoCapture(0)
    print("카메라 켜짐")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # 카메라 초기화 대기
    time.sleep(2)
    
    print("카메라 루프 시작")
    while True:
        ret, frame = cap.read()
        #print("프레임 받음")
        if not ret:
            print("Failed to grab frame")
            break

        und_frame = cv2.undistort(frame, mtx, dist)
        # 마커 검출
        # corners, ids, rejected = detector.detectMarkers(frame_undistorted)
        #print("마커 검출 시작")
        corners, ids, rejected = cv2.aruco.detectMarkers(und_frame, aruco_dict, parameters=aruco_params)
        #print("마커 검출됨")
        # 마커가 검출되면 표시 및 포즈 추정
        if ids is not None:
            # 검출된 마커 표시
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # 각 마커의 포즈 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size, mtx, dist
            )
            
            # 각 마커에 대해 처리
            for i in range(len(ids)):
                # 좌표축 표시
                cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], marker_size/2)
                
                # 마커의 3D 위치 표시
                pos_x = tvecs[i][0][0]
                pos_y = tvecs[i][0][1]
                pos_z = tvecs[i][0][2]
                
                # 회전 벡터를 오일러 각도로 변환
                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]

                T_CAM_BASE = np.array([[ 0.43123891,0.16239037,0.88750345,0.04009533],
                                [-0.86090045,0.36839627,-0.35090541,-0.09004888],
                                [ 0.2699693,0.91537618,0.29866875,0.08887521],
                                [ 0.0,0.0,0.0,1.0]])

                target_mtx = np.eye(4)
                target_mtx[:3, :3] = rot_matrix
                target_mtx[0, 3] = pos_x
                target_mtx[1, 3] = pos_y
                target_mtx[2, 3] = pos_z

                target_position = T_CAM_BASE @ target_mtx

                # print(rot_matrix)
                # print(pos_x, pos_y, pos_z)

                # 마커 정보 표시
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                
                cv2.putText(frame, 
                          "ID: {}".format(ids[i][0]), 
                          (center_x, center_y - 40), 
                          cv2.FONT_HERSHEY_SIMPLEX, 
                          0.5, (0, 0, 0), 1)
                          
                cv2.putText(frame,
                          "Pos: ({:.2f}, {:.2f}, {:.2f})m".format(pos_x, pos_y, pos_z),
                          (center_x, center_y),
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (0, 0, 0), 1)
                          
                cv2.putText(frame,
                          "Rot: ({:.1f}, {:.1f}, {:.1f})deg".format(euler_angles[0], euler_angles[1], euler_angles[2]),
                          (center_x, center_y + 20),
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (0, 0, 0), 1)
                
                # 코너 포인트 표시
                for point in corner:
                    x, y = int(point[0]), int(point[1])
                    cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)

                if cv2.waitKey(1) & 0xFF == ord('r'):
                    inverse_kinematics_mov(dofbot_chain, target_position)
        
        # 프레임 표시
        cv2.imshow('ArUco Marker Detection', frame)
        
        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 리소스 해제
    del Arm
    cap.release()
    cv2.destroyAllWindows()

