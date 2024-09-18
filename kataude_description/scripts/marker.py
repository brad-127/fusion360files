#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_point():
    # ROSノードの初期化
    rospy.init_node('point_publisher')

    # MarkerメッセージをパブリッシュするためのPublisherを作成
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # メッセージが設定されるまでの待機時間
    rospy.sleep(1)

    # Markerメッセージの設定
    marker = Marker()
    marker.header.frame_id = "map"  # 参照するフレーム (例: "map" または "base_link")
    marker.header.stamp = rospy.Time.now()

    marker.ns = "points"
    marker.id = 0

    marker.type = Marker.POINTS
    marker.action = Marker.ADD

    # 点のスケール（大きさ）設定
    marker.scale.x = 0.1  # 点の幅
    marker.scale.y = 0.1  # 点の高さ

    # 点の色を設定 (RGBA)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # 透明度 (1.0で不透明)

    # 任意の座標に点を設定
    p = Point()
    p.x = 1.0  # X座標
    p.y = 2.0  # Y座標
    p.z = 0.0  # Z座標

    marker.points.append(p)

    # ノードが終了しないようにループで保持
    rate = rospy.Rate(10)  # 10Hzでループ
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_point()
    except rospy.ROSInterruptException:
        pass
