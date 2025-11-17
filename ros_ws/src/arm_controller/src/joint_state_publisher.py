#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from Arm_Lib import Arm_Device

def joint_state_talker():
    Arm = Arm_Device()
    pub = rospy.Publisher('joint_states_pub', JointState, queue_size=10)
    rospy.init_node('dofbot_state_publisher', anonymous=True)
    rate = rospy.Rate(500)

    joint_state_msg = JointState()
    while not rospy.is_shutdown():
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.header.frame_id = 'base_link'

        joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper']

        current_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for i in range(6):
            joint_state = Arm.Arm_serial_servo_read(i+1)
            print(joint_state)
            current_position[i] = joint_state

        joint_state_msg.position = current_position

        pub.publish(joint_state_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_talker()
    except rospy.ROSInterruptException:
        pass