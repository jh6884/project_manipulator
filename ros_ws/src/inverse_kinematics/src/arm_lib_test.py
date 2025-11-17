from Arm_Lib import Arm_Device
import time
import cv2


Arm = Arm_Device()

for i in range(1,6):
    init_angle = Arm.Arm_serial_servo_read(i)
    print(init_angle)
    time.sleep(0.5)

# Arm.Arm_serial_set_torque(1)
# Arm.Arm_serial_servo_write6(90,45,45,0,90,0, 1000)
Arm.Arm_serial_set_torque(0)
