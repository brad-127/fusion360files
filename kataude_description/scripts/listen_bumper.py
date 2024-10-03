#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState

def contact_callback(msgs):
    #print('Contqact callback', msgs)
    # メッセージから collision1_name と collision2_name を取得
    for msg in msgs.states:
        collision1_name = msg.collision1_name
        collision2_name = msg.collision2_name
        
        # 右と左が異なるときだけ出力

        if not collision1_name == collision2_name:
            if not (collision1_name.startswith('kataude::') and collision2_name.startswith('kataude::')):
            #rospy.loginfo("Collision Detected:")
            #rospy.loginfo("Collision1 Name: %s", collision1_name)
            #rospy.loginfo("Collision2 Name: %s", collision2_name)
                print(f'Collision Detected({collision1_name}, {collision2_name})')
        else:
            print('Not Detected')

def listener():
    # ノードを初期化
    rospy.init_node('contact_listener')
    print('Contact Listener started')
    # /bumper_states トピックを購読
    rospy.Subscriber("/bumper_states", ContactsState, contact_callback)
    
    # ノードを実行し続ける
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
