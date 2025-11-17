#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    rospy.loginfo(f"Received JointState at time : {msg.header.stamp}")
    if len(msg.position) > 0:
        rospy.loginfo(f"Received Joint Position: {msg.position}")
    else:
        rospy.loginfo("No Received Joint Position")


if __name__ == '__main__':
    rospy.init_node('dofbot_state_subscriber', anonymous=True)
    rospy.Subscriber('joint_states', JointState ,joint_state_callback)
    rospy.spin()
