#!/usr/bin/env python3

import rospy
import geometry_msgs.msg
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from perception.msg import ObjectGoal
from perception.msg import ListGoals
from perception.msg import SceneObject
from perception.msg import ListObjectsVisible
import message_filters
from geometry_msgs.msg import Pose
import math

class PerceptionSystem(object):
    def __init__(self):
        super(PerceptionSystem, self).__init__()
        rospy.init_node('perception_system')
        self.blue_object = ObjectGoal()
        self.yellow_object = ObjectGoal()
        self.red_object = ObjectGoal()
        rospy.Subscriber("/motion_panda/ee_moving", Bool, self.callbackMotion)
        yellow_visible = message_filters.Subscriber("/cylinder_visible", Float64)
        blue_visible = message_filters.Subscriber("/cube_visible", Float64)
        red_visible = message_filters.Subscriber("/ball_visible", Float64)
        self.pub_listgoals = rospy.Publisher("/perception/list_goals",ListGoals,queue_size=10)
        self.pub_listvisible = rospy.Publisher("/perception/list_visible",ListObjectsVisible,queue_size=10)
        ts_visible = message_filters.ApproximateTimeSynchronizer([yellow_visible, blue_visible, red_visible], 10, 0.1, allow_headerless=True)
        ts_visible.registerCallback(self.callbackObjectsVisible)
        blue_position = message_filters.Subscriber('/cube_position', Pose)
        yellow_position = message_filters.Subscriber('/cylinder_position',Pose)
        red_position = message_filters.Subscriber('/ball_position',Pose)
        ts_position = message_filters.ApproximateTimeSynchronizer([blue_position, yellow_position, red_position], 10, 0.1, allow_headerless=True)
        ts_position.registerCallback(self.callbackObjectPosition)
        self.motion_ee = False
        self.motion_blue_object = Bool(False)
        self.motion_green_object = Bool(False)
        self.ref_vector_b = [0.1,0.1]
        self.init_object_b = [0.5,0]
        self.moving_object_b = [0,0]
        self.ref_vector_y = [0.1,0.1]
        self.init_object_y = [0.5,0]
        self.moving_object_y = [0,0]
        self.ref_vector_r = [0.1,0.1]
        self.init_object_r = [0.5,0]
        self.moving_object_r = [0,0]
        self.ax = 100.0 / (3.14-(-3.14)) #param to convert goal angle to field position
        self.bx = 0 - (self.ax*(-3.14)) #same
        self.blue_object.goal = -5.0
        self.yellow_object.goal = -5.0
        self.red_object.goal = -5.0
        self.blue = geometry_msgs.msg.Pose()
        self.yellow = geometry_msgs.msg.Pose()
        self.red = geometry_msgs.msg.Pose()

    def callbackMotion(self,msg):
        self.motion_ee = msg.data

    def callbackObjectsVisible(self,y,b,r):
        self.yellow_object.object = y.data
        self.blue_object.object = b.data
        self.red_object.object = r.data

    def callbackObjectPosition(self,position_b,position_y,position_r):
        self.blue.position.x = position_b.position.x
        self.blue.position.y = position_b.position.y
        self.blue.position.z = position_b.position.z
        self.yellow.position.x = position_y.position.x
        self.yellow.position.y = position_y.position.y
        self.yellow.position.z = position_y.position.z
        self.red.position.x = position_r.position.x
        self.red.position.y = position_r.position.y
        self.red.position.z = position_r.position.z

    def getMotion(self):
        return self.motion_ee

    def initialVectorObject(self):
        self.init_object_b[0] = self.blue.position.x
        self.init_object_b[1] = self.blue.position.y
        self.init_object_y[0] = self.yellow.position.x
        self.init_object_y[1] = self.yellow.position.y
        self.init_object_r[0] = self.red.position.x
        self.init_object_r[1] = self.red.position.y
        self.ref_vector_b[0] = (self.blue.position.x+0.1) - self.blue.position.x
        self.ref_vector_b[1] = (self.blue.position.y+0.1) - self.blue.position.y
        self.ref_vector_y[0] = (self.yellow.position.x+0.1) - self.yellow.position.x
        self.ref_vector_y[1] = (self.yellow.position.y+0.1) - self.yellow.position.y
        self.ref_vector_r[0] = (self.red.position.x+0.1) - self.red.position.x
        self.ref_vector_r[1] = (self.red.position.y+0.1) - self.red.position.y

    def finalVectorObject(self):
        self.moving_object_b[0] = self.blue.position.x
        self.moving_object_b[1] = self.blue.position.y
        self.moving_object_y[0] = self.yellow.position.x
        self.moving_object_y[1] = self.yellow.position.y
        self.moving_object_r[0] = self.red.position.x
        self.moving_object_r[1] = self.red.position.y

    def getPerceivedAngle(self):
        vect_tmp_b = [self.moving_object_b[0] - self.init_object_b[0],self.moving_object_b[1] - self.init_object_b[1]]
        vect_tmp_y = [self.moving_object_y[0] - self.init_object_y[0],self.moving_object_y[1] - self.init_object_y[1]]
        vect_tmp_r = [self.moving_object_r[0] - self.init_object_r[0],self.moving_object_r[1] - self.init_object_r[1]]
        if abs(vect_tmp_b[0]) > 0.02 or abs(vect_tmp_b[1]) > 0.02:
            self.blue_object.goal = self.computeAngle(self.ref_vector_b,vect_tmp_b)
        else:
            self.blue_object.goal = -5.0
        if abs(vect_tmp_y[0]) > 0.02 or abs(vect_tmp_y[1]) > 0.02:
            self.yellow_object.goal = self.computeAngle(self.ref_vector_y,vect_tmp_y)
        else:
            self.yellow_object.goal = -5.0
        if abs(vect_tmp_r[0]) > 0.02 or abs(vect_tmp_r[1]) > 0.02:
            self.red_object.goal = self.computeAngle(self.ref_vector_r,vect_tmp_r)
        else:
            self.red_object.goal = -5.0

    def computeAngle(self,vect1,vect2):
        dot_prod = (vect1[0]*vect2[0]) + (vect1[1]*vect2[1])
        det = (vect1[0]*vect2[1]) - (vect1[1]*vect2[0])
        ang = math.atan2(det,dot_prod)

        return ang

    #convert perceived goal (angle) to neural field position
    def convertAngleToField(self,value):
        field_pos = self.ax * value + self.bx

        return field_pos

    #publish perceived goal (angle) to neural field position
    def publishPerceivedGoals(self):
        list_goals = ListGoals()
        if self.blue_object.goal > -5.0:
            self.blue_object.goal = self.convertAngleToField(self.blue_object.goal)
            list_goals.obj_goals.append(self.blue_object)
        if self.yellow_object.goal > -5.0:
            self.yellow_object.goal = self.convertAngleToField(self.yellow_object.goal)
            list_goals.obj_goals.append(self.yellow_object)
        if self.red_object.goal > -5.0:
            self.red_object.goal = self.convertAngleToField(self.red_object.goal)
            list_goals.obj_goals.append(self.red_object)
        if len(list_goals.obj_goals) > 0:
            self.pub_listgoals.publish(list_goals)
            rospy.sleep(2)
            list_goals = ListGoals()
            self.blue_object.goal = -5
            self.yellow_object.goal = -5
            self.red_object.goal = -5
            list_goals.obj_goals.append(self.blue_object)
            list_goals.obj_goals.append(self.yellow_object)
            list_goals.obj_goals.append(self.red_object)
            self.pub_listgoals.publish(list_goals)

    def publishVisibleObjects(self):
        list_visible = ListObjectsVisible()
        if self.blue_object.object > 0:
            tmp = SceneObject()
            tmp.object_pos.position.x = self.blue.position.x
            tmp.object_pos.position.y = self.blue.position.y
            tmp.object_pos.position.z = self.blue.position.z
            tmp.color = self.blue_object.object
            list_visible.visible_objects.append(tmp)
        if self.yellow_object.object > 0:
            tmp = SceneObject()
            tmp.object_pos.position.x = self.yellow.position.x
            tmp.object_pos.position.y = self.yellow.position.y
            tmp.object_pos.position.z = self.yellow.position.z
            tmp.color = self.yellow_object.object
            list_visible.visible_objects.append(tmp)
        if self.red_object.object > 0:
            tmp = SceneObject()
            tmp.object_pos.position.x = self.red.position.x
            tmp.object_pos.position.y = self.red.position.y
            tmp.object_pos.position.z = self.red.position.z
            tmp.color = self.red_object.object
            list_visible.visible_objects.append(tmp)
        self.pub_listvisible.publish(list_visible)




if __name__ == "__main__":
    percept = PerceptionSystem()
    rate = rospy.Rate(80)   
    motion = True
    while not rospy.is_shutdown():
        if not percept.getMotion():
            if motion:
                percept.getPerceivedAngle()
                percept.publishPerceivedGoals()
                motion = False
            percept.initialVectorObject()
        else:
            percept.finalVectorObject()
            motion = True
            
        percept.publishVisibleObjects()       

        rate.sleep()