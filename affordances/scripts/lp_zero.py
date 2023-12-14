#!/usr/bin/env python3
# license removed for brevity
import rospy
import random
from std_msgs.msg import Float64
from perception.msg import ErrorOG
import sys

def talker():
    pub = rospy.Publisher('/intrinsic/learning_progress', ErrorOG, queue_size=10)
    pubtime = rospy.Publisher('/intrinsic/updating_lp', Float64, queue_size=10)
    err = ErrorOG()
    err.object = float(sys.argv[1])
    err.goal = float(sys.argv[2])
    rospy.init_node('sim', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        t = Float64(1.0)
        tmp = random.random()
        pubtime.publish(t)
        err.value = 0.1
        pub.publish(err)
        print(tmp)
        rospy.sleep(1)
        pubtime.publish(0)
        err.value = 0.0
        pub.publish(err)
        #raw_input("--------------------")
        input("Press Enter to continue...")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
