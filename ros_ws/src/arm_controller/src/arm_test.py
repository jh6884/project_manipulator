from Arm_Lib import Arm_Device
import time
from getkey import getkey, keys
import Jetson.GPIO

Arm = Arm_Device()
time.sleep(.1)
print("device activated")


def main():
        Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)
        first_degree = 90
        second_degree = 90
        third_degree = 90
        fourth_degree = 90
        fifth_degree = 90
        sixth_degree = 90
        time.sleep(1)
        while True:
                key = getkey()
                if key == "p":
                        print("test closed")
                        break
                elif key == "z" and first_degree < 180:
                        first_degree += 1
                        Arm.Arm_serial_servo_write(1, first_degree, 0)
                elif key == "x" and first_degree > 0:
                        first_degree -= 1
                        Arm.Arm_serial_servo_write(1, first_degree, 0)
                elif key == "a" and second_degree < 180:
                        second_degree += 1
                        Arm.Arm_serial_servo_write(2, second_degree, 0)
                elif key == "s" and second_degree > 0:
                        second_degree -= 1
                        Arm.Arm_serial_servo_write(2, second_degree, 0)
                elif key == "q" and third_degree < 180:
                        third_degree += 1
                        Arm.Arm_serial_servo_write(3, third_degree, 0)
                elif key == "w" and third_degree > 0:
                        third_degree -= 1
                        Arm.Arm_serial_servo_write(3, third_degree, 0)
                elif key == "c" and fourth_degree < 180:
                        fourth_degree += 1
                        Arm.Arm_serial_servo_write(4, fourth_degree, 0)
                elif key == "v" and fourth_degree > 0:
                        fourth_degree -= 1
                        Arm.Arm_serial_servo_write(4, fourth_degree, 0)
                elif key == "d" and fifth_degree < 270:
                        fifth_degree += 1
                        Arm.Arm_serial_servo_write(5, fifth_degree, 0)
                elif key == "f" and fourth_degree > 0:
                        fifth_degree -= 1
                elif key == "f" and fourth_degree > 0:
                        fifth_degree -= 1
                        Arm.Arm_serial_servo_write(5, fifth_degree, 0)
                elif key == "e" and sixth_degree < 180:
                        sixth_degree += 1
                        Arm.Arm_serial_servo_write(6, sixth_degree, 0)
                elif key == "r" and sixth_degree > 0:
                        sixth_degree -= 1
                        Arm.Arm_serial_servo_write(6, sixth_degree, 0)

try:
        main()
except KeyboardInterrupt:
        pass

print("test closed")
del Arm

