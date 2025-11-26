import socket
import threading
import cv2, time
from Arm_Lib import Arm_Device
import numpy as np
import datetime

Arm = Arm_Device()

base_pose = [90, 135, 0, 0, 90, 0]
tl_pose = [95, 17, 43, 120, 90, 0]
tr_pose = [72, 17, 43, 120, 90, 0]
bl_pose = [95, 2, 48, 118, 90, 0]
br_pose = [72, 2, 48, 118, 90, 0]
move_pose = [90, 90, 0, 90, 90, 180]
load_pose = [0, 90, 0, 90, 90, 180]

def grip():
    Arm.Arm_serial_servo_write(6, 180, 1500)

def release():
    Arm.Arm_serial_servo_write(6, 0, 1500)

def goto(pose):
    Arm.Arm_serial_servo_write6_array(pose, 1500)

def load(target_pose):
    goto(target_pose)
    time.sleep(1.5)
    grip()
    time.sleep(1.5)
    goto(move_pose)
    time.sleep(1.5)
    goto(load_pose)
    time.sleep(1.5)
    release()
    time.sleep(1.5)
    goto(base_pose)
    time.sleep(1.5)

def start_dofbot_server(host='0.0.0.0', port=50007):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Dofbot Server listening on {host}:{port}. Waiting for load request...")

    try:
        while True:
            client_socket, addr = server_socket.accept()
            print(f"Connection established from {addr} (Turtlebot3)")

            data = client_socket.recv(1024).decode('utf-8')
            if data == "REQUEST_LOAD":
                print("Received signal: REQUEST_LOAD. Initiating Dofbot loading sequence...")
                load(tl_pose)
                responce = "LOAD_COMPLETE"
                client_socket.sendall(responce.encode('utf-8'))
                print("Sent Responce: {}".format(responce))

    except Exception as e:
        print("Error ocurred: {}".format(e))
    finally:
        print("Process closed")
        client_socket.close()

if __name__ == "__main__":
    Arm.Arm_serial_set_torque(1)
    goto(base_pose)
    start_dofbot_server()
