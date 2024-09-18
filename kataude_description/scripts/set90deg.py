#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

def set_joint_angles():
    
    rate = rospy.Rate(10)  # 10 Hz

    joint_state_msg = JointState()
    joint_state_msg.header = Header()
    joint_state_msg.name = [
        'ShoulderJoint1',
        'ShoulderJoint2',
        'ElbowJoint',
        'xm540_1',
        'xm540_2',
        'xm540_3',
        'EndEffecter1',
        'Endeffecter2'
    ]
    joint_state_msg.position = [math.radians(45)] * len(joint_state_msg.name)

    while not rospy.is_shutdown():
        joint_state_msg.header.stamp = rospy.Time.now()
        pub.publish(joint_state_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('set_joint_angles')
        pub = rospy.Publisher('/kataude_controller/joint_states', JointState, queue_size=10)
        set_joint_angles()
    except rospy.ROSInterruptException:
        pass
