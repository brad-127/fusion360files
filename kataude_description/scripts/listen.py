import rospy
from tf2_msgs.msg import TFMessage

def print_all_joint_positions(data):
    for transform in data.transforms:
        child_frame_id = transform.child_frame_id
        position = transform.transform.translation
        print(f"Position of {child_frame_id}: x: {position.x}, y: {position.y}, z: {position.z}")

if __name__ == '__main__':
    rospy.init_node('all_joint_position_listener', anonymous=True)
    rospy.Subscriber("/tf", TFMessage, print_all_joint_positions)
    rospy.spin()
