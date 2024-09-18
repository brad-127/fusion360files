#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ContactsState

def contact_callback(msg):
    rospy.loginfo("Contact detected: %s", msg.states)

def listener():
    rospy.init_node('contact_listener', anonymous=True)
    rospy.Subscriber('/contact_sensor', ContactsState, contact_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
