import rospy
from std_msgs.msg import Float64

def publish_positions():
    # ノードの初期化
    rospy.init_node('position_publisher', anonymous=True)
    
    # パブリッシャの設定
    pub1 = rospy.Publisher('/kataude_controller/ShoulderJoint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/kataude_controller/ShoulderJoint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/kataude_controller/ElbowJoint_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/kataude_controller/xm540_1_position_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/kataude_controller/xm540_2_position_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('/kataude_controller/xm540_3_position_controller/command', Float64, queue_size=10)
    
    # 1秒ごとにメッセージをパブリッシュ
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        # メッセージの作成
        position1 = Float64()
        position1.data = 1.047
        
        position2 = Float64()
        position2.data = 1.047

        position3 = Float64()
        position3.data = 1.047

        position4 = Float64()
        position4.data = 1.047

        position5 = Float64()
        position5.data = 1.047

        position6 = Float64()
        position6.data = 1.047
        
        print("Publishing positions...")

        # メッセージのパブリッシュ
        pub1.publish(position1)
        pub2.publish(position2)
        pub3.publish(position3)
        pub4.publish(position4)
        pub5.publish(position5)
        pub6.publish(position6)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_positions()
    except rospy.ROSInterruptException:
        pass
