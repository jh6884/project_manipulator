import cv2
import numpy as np
import os
import time
import pickle

print(f"현재 opencv 버전: {cv2.__version__}")

def live_aruco_detection():
    # ArUco 검출기 설정
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    
    # 마커 크기 설정 (미터 단위)
    marker_size = 0.05  # 예: 5cm = 0.05m
    
    # 카메라 설정
    cap = cv2.VideoCapture(1)
    
    # 카메라 초기화 대기
    time.sleep(2)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # 마커 검출
        # corners, ids, rejected = detector.detectMarkers(frame_undistorted)
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        
        # 마커가 검출되면 표시 및 포즈 추정
        if ids is not None:
            # 검출된 마커 표시
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # 각 마커의 포즈 추정
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_size
            )
            
            # 각 마커에 대해 처리
            for i in range(len(ids)):
                # 좌표축 표시
                cv2.drawFrameAxes(frame, rvecs[i], tvecs[i], marker_size/2)
                
                # 마커의 3D 위치 표시
                pos_x = tvecs[i][0][0]
                pos_y = tvecs[i][0][1]
                pos_z = tvecs[i][0][2]
                
                # 회전 벡터를 오일러 각도로 변환
                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
                
                # 마커 정보 표시
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                
                cv2.putText(frame, 
                          f"ID: {ids[i][0]}", 
                          (center_x, center_y - 40), 
                          cv2.FONT_HERSHEY_SIMPLEX, 
                          0.5, (0, 0, 0), 2)
                          
                cv2.putText(frame,
                          f"Pos: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})m",
                          (center_x, center_y),
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (0, 0, 0), 2)
                          
                cv2.putText(frame,
                          f"Rot: ({euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f})deg",
                          (center_x, center_y + 20),
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (0, 0, 0), 2)
                
                # 코너 포인트 표시
                for point in corner:
                    x, y = int(point[0]), int(point[1])
                    cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)
        
        # 프레임 표시
        cv2.imshow('ArUco Marker Detection', frame)
        
        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 리소스 해제
    cap.release()
    cv2.destroyAllWindows()

def main():
    live_aruco_detection()

if __name__ == "__main__":
    main()