#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import threading

class JointStatePublisher:
    def __init__(self):
        self.kakudo = 45
        self.kakudo_lock = threading.Lock()
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def set_joint_angles(self):
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
        
        while not rospy.is_shutdown():
            with self.kakudo_lock:
                joint_state_msg.position = [math.radians(self.kakudo)] * len(joint_state_msg.name)
            
            joint_state_msg.header.stamp = rospy.Time.now()
            self.pub.publish(joint_state_msg)
            self.rate.sleep()

    def update_kakudo(self, new_value):
        with self.kakudo_lock:
            self.kakudo = new_value

def main(joint_publisher):
    
    
    
    # set_joint_anglesメソッドを呼び出してjoint anglesを設定する
    joint_publisher.set_joint_angles()

if __name__ == '__main__':
    rospy.init_node('set_joint_angles')
    joint_publisher = JointStatePublisher()
    main(joint_publisher)
