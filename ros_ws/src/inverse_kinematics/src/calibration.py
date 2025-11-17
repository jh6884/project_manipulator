import cv2, os, glob
import numpy as np
import time

GSTRAMER_PIPELINE = (
    "v4l2src device=/dev/video0 ! video/x-raw, width=320, height=240, framerate=15/1 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
)

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
squares_ver = 7
squares_hor = 10

square_length = 0.015
marker_length = 0.011

board = cv2.aruco.CharucoBoard((squares_hor, squares_ver), square_length, marker_length, ARUCO_DICT)

img_dir = "./images"

def image_capture():
#    cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(GSTRAMER_PIPELINE, cv2.CAP_GSTREAMER)
    img_count = 0
    count = 0
    if not cap.isOpened():
        print("카메라 안열림")
        exit()

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids, rejected = cv2.aruco.detectMarkers(frame, ARUCO_DICT)
        
        if ids is not None:
            charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, frame, board
            )
            frame = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            if charuco_corners is not None and charuco_ids is not None:
                frame = cv2.aruco.drawDetectedCornersCharuco(frame.copy(), charuco_corners, charuco_ids)

        cv2.imshow("camera calibration", frame)
        key = cv2.waitKey(1) & 0xFF
        count += 1
        print(count)
        if key == ord('c'):
            filename = os.path.join(img_dir, f"img_{img_count:04d}.png")
            cv2.imwrite(filename, frame)
            print(f"이미지 저장됨: {filename}")
            img_count += 1
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return img_count

def calibrate_camera(image_files):
    all_charuco_corners = []
    all_charuco_ids = []

    img_size = None
    detectorParams = cv2.aruco.DetectorParameters()

    print("\n-캘리브레이션 실행 시작-")
    for filename in image_files:
        img = cv2.imread(filename)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = gray.shape[::1]
        
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=detectorParams)

        if ids is not None:
            charuco_corners, charuco_ids = cv2.aruco.interpolateConrersCharuco(
                corners, ids, gray, board
            )

            if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) > 5:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                print(f"분석 완료: {filename} \n검출 코너: {len(charuco_corners)}")

    if not all_charuco_corners:
        print("Error: No suitable charuco corners found in any image")
        return None, None, None
    
    print(f"\n총 {len(all_charuco_corners)}장의 이미지로 캘리브레이션 실행")

    retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_charuco_corners, all_charuco_ids, board, img_size, None, None
    )

    return cameraMatrix, distCoeffs, retval

if __name__ == "__main__":
    num_captured = image_capture()

    image_files = glob.glob(os.path.join(img_dir, "*.png"))
    if image_files:
        mtx, dist, error = calibrate_camera(image_files)

        if mtx is not None:
            print("\n왜곡 보정 테스트")

            test_img_path = image_files[-1]
            test_img = cv2.imread(test_img_path)
            h, w = test_img.shape[:2]

            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

            dst = cv2.undistort(test_img, mtx, dist, None, newcameramtx)

            test_img_resized = cv2.resize(test_img, (int(w/2), int(h/2)))
            dst_resized = cv2.resize(dst, (int(w/2), int(h/2)))

            comparison_img = np.hstack((test_img_resized, dst_resized))
            cv2.putText(comparison_img, 'Original', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(comparison_img, 'Undistorted', ((int(w/2)+10), 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Calib test', comparison_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            print("테스트 종료")
    else:
        print("Error: 이미지가 없습니다.")