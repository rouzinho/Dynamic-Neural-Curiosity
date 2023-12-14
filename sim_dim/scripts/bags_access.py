#!/usr/bin/env python3
import sys
import copy
import rospy
import rosbag
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
import rospy
import numpy as np
from math import pi
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import os.path
from os import path
from std_msgs.msg import Float64

class DataAccess(object):
    def __init__(self):
        super(DataAccess, self).__init__()
       


    def printBag(self,name):
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['position']):
            #print(msg.header.stamp.to_sec())
            print(msg.position)
            print("-----")
        bag.close()

    def printJointStatesBag(self,name):
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['js_pos']):
            print("test")
            print(msg.name)
            print(msg.position)
            print("-----")
        bag.close()
    
    def printBagEE(self,name):
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['ee_pos']):
            print(msg.position)
            print("-----")
        bag.close()

    def printBagCube(self,name):
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['cube_pos']):
            print(msg.position)
            print("-----")
        bag.close()

    def printBagObject(self,name):
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['object_pos']):
            print(msg.position)
            print("-----")
        bag.close()

    def printDMP(self,name):
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['dmp_pos']):
            print(msg)
            print("-----")
        bag.close()

    def printAllBags(self,name):
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['ee_pos','cube_pos']):
            #if topic == "ee_pos":
            print(msg.position)
            #if topic == "cube_pos":
            #print(msg.position)
            print("-----")
        bag.close()

    def cleanEEBags(self):
        fpos = True
        first = True
        try:
            for count in range(1,2):
                name_open = "rosbags/dataset/ee_raw/" + str(count) + "_motion_EE.bag"
                name_new = "rosbags/dataset/ee_clean/" + str(count) + "_motion_EE.bag"
                name_fpos = "rosbags/dataset/ee_clean/" + str(count) + "_fpos_motion_EE.bag"
                bag_open = rosbag.Bag(name_open)
                bag_new = rosbag.Bag(name_new,"w")
                bag_fpos = rosbag.Bag(name_fpos,"w")
                for topic, msg, t in bag_open.read_messages(topics=['position']):
                    if first == False:
                        bag_new.write("position",msg)
                    if (msg.position.z <= 0.141 and msg.position.z > 0.138) and first == True:
                        first = False
                        if fpos == True:
                            bag_fpos.write("position",msg)
                            bag_new.write("position",msg)
                            fpos = False
                bag_open.close()
                bag_new.close()
                bag_fpos.close()
                fpos = True
                first = True
        finally:
            fpos = False
    #divide bags between ee and cube position
    # clean ee bags by removing repeated same position
    # remove objects bags first repeated positions by the same occasion
    # generated bags are the same size but not in sync
    def cleanAllBags(self):
        fpos = True
        first = True
        try:
            for count in range(0,78):
                name_open = "rosbags/dataset/raw_bags/" + str(count) + "_motion.bag"
                name_newEE = "rosbags/dataset/raw_bags/" + str(count) + "_motionEE.bag"
                name_newCube = "rosbags/dataset/raw_bags/" + str(count) + "_motionCube.bag"
                #name_fpos = "rosbags/dataset/clean_bags/" + str(count) + "_fpos_motions.bag"
                bag_open = rosbag.Bag(name_open)
                bag_newEE = rosbag.Bag(name_newEE,"w")
                bag_cube = rosbag.Bag(name_newCube,"w")
                for topic, msg, t in bag_open.read_messages(topics=['ee_pos','cube_pos']):
                    if topic == "ee_pos" and first == False:
                        bag_newEE.write("ee_pos",msg)
                    if topic == "cube_pos" and first == False:
                        bag_cube.write("cube_pos",msg)
                    if topic == "ee_pos" and first == True: 
                        if (msg.position.z <= 0.141 and msg.position.z > 0.138):
                            first = False
                            bag_newEE.write("ee_pos",msg)
                    #if topic == "cube_pos" and fpos == True:
                        #bag_cube.write("cube_pos",msg)
                        #fpos = False
                bag_open.close()
                bag_newEE.close()
                bag_cube.close()
                fpos = True
                first = True
        finally:
            fpos = False

    def syncbags(self):
        first = True
        for count in range(0,78):
            x = 0
            y = 0
            old_x = 0
            old_y = 0

            cube_open = "rosbags/dataset/raw_bags/" + str(count) + "_motionCube.bag"
            new_cube = "rosbags/dataset/clean_bags/" + str(count) + "_motionCube.bag"
            ee_open = "rosbags/dataset/raw_bags/" + str(count) + "_motionEE.bag"
            new_ee = "rosbags/dataset/clean_bags/" + str(count) + "_motionEE.bag"
            cube_bag = rosbag.Bag(cube_open)
            cube_newbag = rosbag.Bag(new_cube,"w")
            for topic, msg, t in cube_bag.read_messages(topics=['cube_pos']):
                x = msg.position.x
                y = msg.position.y
                if first == True:
                    cube_newbag.write("cube_pos",msg)
                    first = False
                    count += 1
                else:
                    if x == old_x and y == old_y:
                        count += 1
                    else:
                        cube_newbag.write("cube_pos",msg)
                old_x = x
                old_y = y
            cube_bag.close()
            cube_newbag.close()
            first = True
            ee_bag = rosbag.Bag(ee_open)
            size_ee = ee_bag.get_message_count()
            size_ee = size_ee - count
            tmp = 0
            ee_bag = rosbag.Bag(ee_open)
            ee_newbag = rosbag.Bag(new_ee,"w")
            for topic, msg, t in ee_bag.read_messages(topics=['ee_pos']):
                if tmp <= size_ee:
                    ee_newbag.write("ee_pos",msg)
                    tmp += 1
            ee_bag.close()
            ee_newbag.close()
            count = 0
            
            



    def cleanObjectBags(self):
        first = True
        try:
            for count in range(0,1):
                name_open = "rosbags/dataset/object_raw/" + str(count) + "_motion_object.bag"
                name_new = "rosbags/dataset/object_clean/" + str(count) + "_motion_object.bag"
                name_fpos = "rosbags/dataset/object_clean/" + str(count) + "_objectfpos.bag"
                bag_open = rosbag.Bag(name_open)
                bag_new = rosbag.Bag(name_new,"w")
                bag_fpos = rosbag.Bag(name_fpos,"w")
                first = True
                old_msg = geometry_msgs.msg.Pose()
                for topic, msg, t in bag_open.read_messages(topics=['position']):
                    if first == True:
                        old_msg = msg
                        bag_new.write("position",msg)
                        bag_fpos.write("position",msg)
                        first = False
                    if (old_msg.position.x != msg.position.x) or (old_msg.position.y != msg.position.y):
                        bag_new.write("position",msg)
                    old_msg = msg
                bag_open.close()
                bag_new.close()
                bag_fpos.close()
        finally:
            pass

    def genInitialObjectPosition(self,n,new_n):
        object_init = geometry_msgs.msg.Pose()
        name_open = n
        name_new = new_n
        bag_open = rosbag.Bag(name_open)
        first = True
        for topic, msg, t in bag_open.read_messages(topics=['cube_pos']):
            if first == True:
                object_init.position.x = msg.position.x
                object_init.position.y = msg.position.y
                object_init.position.z = msg.position.z
                first = False
        bag_open.close()
        bag_new = rosbag.Bag(name_new,"w")
        bag_new.write("object_pos",object_init)
        bag_new.close()
        

    def genLastEEPosition(self,n,new_n):
        pose_goal = geometry_msgs.msg.Pose()
        name_open = n
        name_new = new_n
        bag_open = rosbag.Bag(name_open)
        bag_new = rosbag.Bag(name_new,"w")
        for topic, msg, t in bag_open.read_messages(topics=['ee_pos']):
            pose_goal = msg
        bag_new.write("ee_pos",pose_goal)
        bag_open.close()
        bag_new.close()

    def genLastObjectPosition(self,n,new_n):
        object_goal = geometry_msgs.msg.Pose()
        name_open = n
        name_new = new_n
        bag_open = rosbag.Bag(name_open)
        bag_new = rosbag.Bag(name_new,"w")
        for topic, msg, t in bag_open.read_messages(topics=['cube_pos']):
            object_goal.position.x = msg.position.x
            object_goal.position.y = msg.position.y
            object_goal.position.z = msg.position.z
        bag_new.write("object_pos",object_goal)
        bag_open.close()
        bag_new.close()



if __name__ == '__main__':
    try:
        controller = DataAccess()
        
        #for i in range(0,19):
        #    motion_ee = "rosbags/experiment/goals/3.10/" + str(i) + "_motionEE.bag"
        #    motion_object = "rosbags/experiment/goals/3.10/" + str(i) + "_motionCube.bag"
        #    last_ee = "rosbags/experiment/goals/2.10/" + str(i) + "_goalmotion_EE.bag"
        #    fpos = "rosbags/experiment/goals/2.10/" + str(i) + "_objectfpos.bag"
        #    lpos = "rosbags/experiment/goals/2.10/" + str(i) + "_objectlastpos.bag"
            #controller.genLastEEPosition(motion_ee,last_ee)
            #controller.genInitialObjectPosition(motion_object,fpos)
            #controller.genLastObjectPosition(motion_object,lpos)
        
        #controller.genLastEEPosition()
        #controller.printBag("rosbags/dataset/test/medium_totop_EE.bag")
        #controller.genCurrentObjectPosition()
        #controller.printBagCube("rosbags/experiment/motion_Object.bag")
        #print("OBJECT FIRST POSITION")
        #controller.genInitialObjectPosition("rosbags/experiment/motion_Object.bag","rosbags/experiment/goals/2.355/0_objectfpos.bag")
        #print("LAST POSITION")
        #controller.printBagObject("rosbags/experiment/datas/object-60/goal-62/0_objectlastpos.bag")
        #print("FIRST POSITION")
        #controller.printBagObject("rosbags/experiment/datas/object-60/goal-62/0_objectfpos.bag")
        print("GOAL EE")
        controller.printBagEE("rosbags/experiment/motion_EE.bag")
        print("-_-_-_-_-_-_-_-_-_-_-_-_--")
        #print("LAST POSITION")
        #controller.printBagObject("rosbags/experiment/datas/object-60/goal-62/1_objectlastpos.bag")
        #print("FIRST POSITION")
        #controller.printBagObject("rosbags/experiment/datas/object-60/goal-62/1_objectfpos.bag")
        #print("GOAL EE")
        #controller.printBagEE("rosbags/experiment/datas/object-60/goal-62/1_goalmotion_EE.bag")
        #controller.printBagCube("rosbags/experiment/goals/2.10/0_objectlastpos.bag")
        #controller.printBagEE("rosbags/experiment/goals/2.10/0_goalmotion_EE.bag")
        #print("LAST POSITION")
        #controller.printBagCube("rosbags/experiment/goals/2.3236/0_objectlastpos.bag")
        #controller.printDMP("rosbags/experiment/dmp/dmp_2.3236.bag")
        #print("LAST")
        #controller.printBagEE("rosbags/experiment/goals/2.3236/0_goalmotion_EE.bag")
        #print("EE")
        #controller.printBagEE("rosbags/experiment/motion_EE.bag")
        

        
        
       
    except rospy.ROSInterruptException:
        pass


