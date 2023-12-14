#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
import rospy
import numpy as np
import math
from numpy import arccos, array
from numpy.linalg import norm
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Bool
## END_SUB_TUTORIAL

class AngleDetector(object):
    def __init__(self):
        super(AngleDetector, self).__init__()
        rospy.init_node('angle', anonymous=True)
        pub_angle = rospy.Publisher('/cylinder_angle', Float64, queue_size=10)
        rospy.Subscriber('/ee_moving', Bool, self.callbackMove)
        rospy.Subscriber('/cylinder_position', Pose, self.callbackObjectPosition)
        rospy.Subscriber('/cylinder_motion', Bool, self.callbackMotionObject)
        self.pub = pub_angle
        self.angle = Float64(0)
        self.move_ee = Bool(False)
        self.motion_object = Bool(False)
        self.ref_vector = [0.1,0.1]
        self.init_object = [0.5,0]
        self.moving_object = [0,0]
    
  
    def callbackMove(self,moving):
        self.move_ee.data = moving.data
        

    def callbackObjectPosition(self,object):
        if self.motion_object.data == False:
            #print("object not moving init position")
            self.init_object[0] = object.position.x
            self.init_object[1] = object.position.y
            self.ref_vector[0] = (object.position.x+0.1) - object.position.x
            self.ref_vector[1] = (object.position.y+0.1) - object.position.y
        if self.motion_object.data == True:
            #print("object moving retriving last position")
            self.moving_object[0] = object.position.x
            self.moving_object[1] = object.position.y
            if self.move_ee.data == True:
                vect_tmp = [self.moving_object[0] - self.init_object[0],self.moving_object[1] - self.init_object[1]]
                self.angle.data = self.computeAngle(self.ref_vector,vect_tmp)

        
    def callbackMotionObject(self,motion):
        self.motion_object.data = motion.data
        #if self.motion_object.data == True:
        #    vect_tmp = [object.position.x - self.init_object[0],object.position.y - self.init_object[1]]
        #    self.angle.data = self.calcAngle(self.ref_vector,vect_tmp)

    def calcAngle(self,vect1,vect2):
        dot_prod = (vect1[0]*vect2[0]) + (vect1[1]*vect2[1])
        norm_v1 = math.sqrt((vect1[0]**2)+(vect1[1]**2))
        norm_v2 = math.sqrt((vect2[0]**2)+(vect2[1]**2))
        tmp = dot_prod/(norm_v1*norm_v2)
        a = math.acos(tmp)
        return a

    def publishAngle(self):
        self.pub.publish(self.angle)

    def computeAngle(self,vect1,vect2):
        dot_prod = (vect1[0]*vect2[0]) + (vect1[1]*vect2[1])
        det = (vect1[0]*vect2[1]) - (vect1[1]*vect2[0])
        ang = math.atan2(det,dot_prod)

        return ang



def main():
  try:
    ag = AngleDetector()
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        ag.publishAngle()
        rate.sleep()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
