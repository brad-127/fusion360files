import rospy
from std_msgs.msg import Float64
import random  # random モジュールをインポート
import math

def publish_positions():
    # ノードの初期化
    rospy.init_node('position_publisher', anonymous=True)
    
    # パブリッシャの設定
    pub1 = rospy.Publisher('/migiude_controller/RShoulderJoint1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/migiude_controller/RShoulderJoint2_position_controller/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/migiude_controller/RElbowJoint_position_controller/command', Float64, queue_size=10)
    pub4 = rospy.Publisher('/migiude_controller/Rxm540_1_position_controller/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/migiude_controller/Rxm540_2_position_controller/command', Float64, queue_size=10)
    pub6 = rospy.Publisher('/migiude_controller/Rxm540_3_position_controller/command', Float64, queue_size=10)
    
    pub7 = rospy.Publisher('/hidariude_controller/LShoulderJoint1_position_controller/command', Float64, queue_size=10)
    pub8 = rospy.Publisher('/hidariude_controller/LShoulderJoint2_position_controller/command', Float64, queue_size=10)
    pub9 = rospy.Publisher('/hidariude_controller/LElbowJoint_position_controller/command', Float64, queue_size=10)
    pub10 = rospy.Publisher('/hidariude_controller/Lxm540_1_position_controller/command', Float64, queue_size=10)
    pub11 = rospy.Publisher('/hidariude_controller/Lxm540_2_position_controller/command', Float64, queue_size=10)
    pub12 = rospy.Publisher('/hidariude_controller/Lxm540_3_position_controller/command', Float64, queue_size=10)
    
    # 1秒ごとにメッセージをパブリッシュ
    rate = rospy.Rate(10)  # 1Hz

    while not rospy.is_shutdown():
        # メッセージの作成
        position1 = Float64()
        position1.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値

        position2 = Float64()
        position2.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値

        position3 = Float64()
        position3.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値

        position4 = Float64()
        position4.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値

        position5 = Float64()
        position5.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値

        position6 = Float64()
        position6.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値
        
        position7 = Float64()
        position7.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値
        
        position8 = Float64()
        position8.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値
        
        position9 = Float64()
        position9.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値
        
        position10 = Float64()
        position10.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値
        
        position11 = Float64()
        position11.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値
        
        position12 = Float64()
        position12.data = random.uniform(-math.pi/2, math.pi/2)  # -πからπの範囲でランダムな値
        
        print(f"Publishing positions... Position1: {position1.data} radians")

        # メッセージのパブリッシュ
        pub1.publish(position1)
        pub2.publish(position2)
        pub3.publish(position3)
        pub4.publish(position4)
        pub5.publish(position5)
        pub6.publish(position6)

        pub7.publish(position7)
        pub8.publish(position8)
        pub9.publish(position9)
        pub10.publish(position10)
        pub11.publish(position11)
        pub12.publish(position12)

        #rate.sleep()
        ## 5秒ごとにランダムな値に変更
        rospy.sleep(2)
        
if __name__ == '__main__':
    try:
        publish_positions()
    except rospy.ROSInterruptException:
        pass
