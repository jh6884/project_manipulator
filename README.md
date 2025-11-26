디렉토리 구조 및 설명  
  
ros_ws  
├src  
│  ├arm_controller  
│  │   ├launch  
│  │   │   └arm_controller.launch / 하단 joint_state 관련 topic 구독/발행용 패키지 런치 파일  
│  │   └src  
│  │   │    ├joint_state_publisher.py / 로봇 팔 joint 상태 발행  
│  │   │    ├joint_state_subscriber.py / 로봇 팔 joint 상태 구독  
│  │   │    └arm_test.py / 로봇 팔 테스트용 코드  
│  ├inverse_kinematics  
│  │   └src  
│  │   │    ├arm_lib_test.py / 로봇 팔 라이브러리 테스트  
│  │   │    ├arucotest.py / 아루코 마커 인식 테스트  
│  │   │    ├calibration.py / 카메라 캘리브레이션  
│  │   │    ├fktest.py / 정기구학 모듈 테스트  
│  │   │    ├iktest.py / 역기구학 모듈 테스트  
│  │   │    ├main_cam.py / 3차원 좌표 기반 기구학 계산, 이동  
│  │   │    ├main.py / 로봇 pose 기반 물체 이동 코드 (시연)  
│  │   │    └robot_base_calibration.py / 카메라와 로봇 위치 보정  
