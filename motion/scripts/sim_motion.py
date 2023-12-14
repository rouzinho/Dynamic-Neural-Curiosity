#!/usr/bin/env python3
# license removed for brevity
import rospy
import random
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose


pose_object = geometry_msgs.msg.Pose()


def callbackPos(pos):
    pose_object.position.x = pos.position.x
    pose_object.position.y = pos.position.y
    pose_object.position.z = pos.position.z
    pose_object.orientation.x = 0.93
    pose_object.orientation.y = -0.38
    pose_object.orientation.z = 0
    pose_object.orientation.w = 0


if __name__ == '__main__':
    rospy.init_node('sim', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    pub = rospy.Publisher('/path_ee', Pose, queue_size=1)
    sub = rospy.Subscriber('/cylinder_position', Pose, callbackPos)
    pose_object.position.y = pose_object.position.y + 0.08
    pub.publish(pose_object)
    time.sleep(2)
    pose_object.position.y = pose_object.position.y - 0.08
    pub.publish(pose_object)