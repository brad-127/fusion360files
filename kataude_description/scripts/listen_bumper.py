#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactState

def contact_callback(msg):
    # メッセージから collision1_name と collision2_name を取得
    collision1_name = msg.collision1_name
    collision2_name = msg.collision2_name
    
    # 右と左が異なるときだけ出力
    if collision1_name != collision2_name:
        rospy.loginfo("Collision Detected:")
        rospy.loginfo("Collision1 Name: %s", collision1_name)
        rospy.loginfo("Collision2 Name: %s", collision2_name)

def listener():
    # ノードを初期化
    rospy.init_node('contact_listener', anonymous=True)
    
    # /bumper_states トピックを購読
    rospy.Subscriber("/bumper_states", ContactState, contact_callback)
    
    # ノードを実行し続ける
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
