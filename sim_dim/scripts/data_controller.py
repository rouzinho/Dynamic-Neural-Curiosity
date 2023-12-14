#!/usr/bin/env python
import sys
import copy
import rospy
import rosbag
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
roslib.load_manifest('dmp')
from dmp.srv import *
from dmp.msg import *
import rospy
import message_filters
import numpy as np
from math import pi
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import os.path
from os import path
from perception.msg import ObjectGoal
from perception.msg import ListGoals


class DataController(object):
    def __init__(self):
        super(DataController, self).__init__()
        rospy.init_node('controller', anonymous=True)
        self.ee_pose = geometry_msgs.msg.Pose()
        self.blue_pos = geometry_msgs.msg.Pose()
        self.green_pos = geometry_msgs.msg.Pose()
        rospy.Subscriber("/motion_panda/ee_moving", Bool, self.callbackMotion)
        rospy.Subscriber("/proprioception_ee", Pose, self.callbackProprioception)
        rospy.Subscriber("/cylinder_position", Pose, self.callbackBluePosition)
        rospy.Subscriber("/cylinder_green_position", Pose, self.callbackGreenPosition)
        rospy.Subscriber("/data_controller/object_goal", ObjectGoal, self.callbackObjectGoal)
        explor = message_filters.Subscriber("/hebbian_server/reward", Float64)
        exploi = message_filters.Subscriber("/hebbian_server/dmp", Float64)
        ts_mode = message_filters.ApproximateTimeSynchronizer([explor, exploi], 10, 0.1, allow_headerless=True)
        ts_mode.registerCallback(self.callbackMode)
        self.is_moving = False
        self.count = 0
        self.action = False
        self.object_first = True
        self.name = []
        self.dims = 3 
        self.dt = 1.0
        self.K_gain = 100              
        self.D_gain = 2.0 * np.sqrt(self.K_gain)      
        self.num_bases = 4
        self.angle = Float64(-1.0)
        self.list_angles = [] 
        self.active_list_goals = ListGoals()
        self.list_goals = []
        self.list_samples = []
        self.og = ObjectGoal()
        self.og.object = -1
        self.og.goal = -1 
        self.explore = 0.0
        self.exploit = 0.0

    def callbackMotion(self,data):
        self.is_moving = data.data
        if self.is_moving == True:
            #self.count = self.count + 1
            pass
        else:
            self.object_first = True

    def callbackProprioception(self,ee):
        self.ee_pose.position.x = ee.position.x
        self.ee_pose.position.y = ee.position.y
        self.ee_pose.position.z = ee.position.z
        
    def callbackBluePosition(self,cube):
        self.blue_pos.position.x = cube.position.x
        self.blue_pos.position.y = cube.position.y
        self.blue_pos.position.z = cube.position.z

    def callbackGreenPosition(self,cube):
        self.green_pos.position.x = cube.position.x
        self.green_pos.position.y = cube.position.y
        self.green_pos.position.z = cube.position.z

    def callbackActiveObjects(self,lg):
        self.active_list_goals.obj_goals = []
        tmp_og = ObjectGoal()
        for i in range(len(lg.obj_goals)):
            tmp_og.object = lg.obj_goals[i].object
            tmp_og.goal = lg.obj_goals[i].goal
            self.active_list_goals.obj_goals.append(tmp_og)

    def callbackObjectGoal(self,obg):
        self.og.object = obg.object
        self.og.goal = obg.goal

    def callbackMode(self,explor,exploi):
        self.explore = explor.data
        self.exploit = exploi.data

    def formDatasEE(self,n):
        name = n
        tot = []
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['ee_pos']):
            ee = []
            x = msg.position.x
            ee.append(x)
            y = msg.position.y
            ee.append(y)
            z = msg.position.z
            ee.append(z)
            tot.append(ee)

        bag.close() 

        return tot

    def makeLFDRequest(self,traj):
        demotraj = DMPTraj()
            
        for i in range(len(traj)):
            pt = DMPPoint();
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(self.dt*i)
                
        k_gains = [self.K_gain]*self.dims
        d_gains = [self.D_gain]*self.dims
            
        print ("Starting LfD...")
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, self.num_bases)
        except rospy.ServiceException:
            print("Service call failed: %s")
            print("LfD done")    
                
        return resp;

    #write EE motion in a bag
    def writeBagEE(self,n):
        name = n
        exist = path.exists(name)
        opening = ""
        if(exist == True):
            opening = "a"
        else:
            opening = "w"
        bag = rosbag.Bag(name, opening)
        try:
            bag.write("ee_pos",self.ee_pose)
        finally:
            bag.close()

    #write blue object motion in a bag
    def writeBagBlueObjectPos(self,n):
        name = n
        exist = path.exists(name)
        opening = ""
        if(exist == True):
            opening = "a"
        else:
            opening = "w"
        bag = rosbag.Bag(name, opening)
        try:
            bag.write("object_pos",self.blue_pos)
        finally:
            bag.close()

    #write green object motion in a bag
    def writeBagGreenObjectPos(self,n):
        name = n
        exist = path.exists(name)
        opening = ""
        if(exist == True):
            opening = "a"
        else:
            opening = "w"
        bag = rosbag.Bag(name, opening)
        try:
            bag.write("object_pos",self.blue_pos)
        finally:
            bag.close()

    # name the fresh bags since there might be a slight shift <2 . Avoid having 2 samples separated when they should be together
    def renameFreshBag(self,obg):
        inlist = False
        for i in range(0,len(self.list_samples)):
            tmp_o = abs(self.list_samples[i].object - obg.object)
            tmp_g = abs(self.list_samples[i].goal - obg.goal)
            #print("tmp_o : ",tmp_o)
            #print("tmp_g : ",tmp_g)
            if tmp_o <= 4:
                if tmp_g <= 4:
                    obg.object = self.list_samples[i].object
                    obg.goal = self.list_samples[i].goal
                    inlist = True
        if inlist == False:
            self.list_samples.append(copy.deepcopy(obg))

        #print("list samples : ",self.list_samples)

        return obg

    #create  first and last position of object + goal position of EE and dispatch all of them in appropriate repositories for machine learning
    def dispatchBags(self,obg,name_ee):
        obj = ""
        if int(obg.object) <= 62 and  int(obg.object) >= 58 :
            obj = "60"
        if int(obg.object) <= 26 and  int(obg.object) >= 22 :
            obj = "24"
        name_object = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_"+ obj +".bag"
        repo = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/datas/object-"+ obj +"/goal-"+str(int(obg.goal))+"/"
        new_initpos_object = ""
        new_finalpos_object = ""
        new_goal_ee = ""
        exist = path.isdir(repo)
        if exist == False:
            os.makedirs(repo)
        size = len(os.listdir(repo))
        if size == 0:
            new_initpos_object = repo + "0_objectfpos.bag"
            new_finalpos_object = repo + "0_objectlastpos.bag"
            new_goal_ee = repo + "0_goalmotion_EE.bag"
        else:
            size = size / 3
            new_initpos_object = repo + str(int(size)) + "_objectfpos.bag"
            new_finalpos_object = repo + str(int(size)) + "_objectlastpos.bag"
            new_goal_ee = repo + str(int(size)) + "_goalmotion_EE.bag"
        
        self.genInitialObjectPosition(name_object,new_initpos_object)
        self.genLastObjectPosition(name_object,new_finalpos_object)
        self.genLastEEPosition(name_ee,new_goal_ee)
        
        
    def genInitialObjectPosition(self,n,new_n):
        object_init = geometry_msgs.msg.Pose()
        name_open = n
        name_new = new_n
        bag_open = rosbag.Bag(name_open)
        first = True
        for topic, msg, t in bag_open.read_messages(topics=['object_pos']):
            if first == True:
                object_init = msg
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
        for topic, msg, t in bag_open.read_messages(topics=['object_pos']):
            object_goal = msg
        bag_new.write("object_pos",object_goal)
        bag_open.close()
        bag_new.close()

    def removeTmpBag(self,n):
        os.remove(n)

    def getMoving(self):
        return self.is_moving

    def getObjectFirst(self):
        return self.object_first

    def setObjectFirst(self,status):
        self.object_first = status

    def getExplore(self):
        return self.explore
    
    def getExploit(self):
        return self.exploit

    def resetObjectGoal(self):
        self.og.object = -1
        self.og.goal = -1      

    def getObjectGoal(self):
        return self.og      



    #write the DMP in a bag
    def writeDMPBag(self,data,n):
        name = n
        exist = path.exists(name)
        opening = ""
        if(exist == False):
            opening = "w"
        else:
            opening = "a"
        bag = rosbag.Bag(name, opening)
        try:
            bag.write("dmp_pos",data)
        finally:
            bag.close()

if __name__ == '__main__':
    try:
        name_object_blue = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_60.bag"
        name_object_green = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_24.bag"
        name_ee = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_EE.bag"
        motion = False
        keep_record = False
        mode = False
        set_mode = 0
        controller = DataController()
        tmp_ob = ObjectGoal()
        tmp_ob.object = -1
        tmp_ob.goal = -1
        ob = ObjectGoal()
        ob.object = -1
        ob.goal = -1
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            #if EE is performing an action
            if controller.getMoving() == True:
                #record the action, the object motion and check if the goal angle is new
                motion = True
                if mode == False:
                    if controller.getExplore() > 0.8:
                        set_mode = 1
                        #print("mode explore")
                    if controller.getExploit() > 0.8:
                        set_mode = 2
                        #print("mode exploit")
                    mode = True
                #print("writing pos")
                controller.writeBagEE(name_ee)
                controller.writeBagBlueObjectPos(name_object_blue)
                controller.writeBagGreenObjectPos(name_object_green) 
                tmp_ob = controller.getObjectGoal()
                if keep_record == False:
                    if tmp_ob.object != -1 and int(tmp_ob.object) != 50:
                        print("DATA CONTROLLER : GOOD ACTION")
                        print("DATA CONTROLLER : got perception ",tmp_ob)
                        keep_record = True
                        ob.object = tmp_ob.object
                        ob.goal = tmp_ob.goal
                        tmp_ob.object = -1
                        tmp_ob.goal = -1
                        
            #if EE is not moving and the last action was meaningful, turn it into a DMP and dispatch the EE goal and last object position into apropriate repositories
            if controller.getMoving() == False and motion == True:
                #print(ob)
                if keep_record == True:
                    #detector is resilient but there might be a slight difference
                    obj_goal = controller.renameFreshBag(ob)
                    print("DATA CONTROLLER : modified obj ",obj_goal)
                    name_object = str(int(obj_goal.object))
                    name_goal = str(int(obj_goal.goal))
                    if set_mode == 1:
                        print("DATA CONTROLLER : write DMP")
                        name_dmp = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/dmp/dmp_" + name_object + "_" + name_goal + ".bag"
                        traj = controller.formDatasEE(name_ee)
                        plan = controller.makeLFDRequest(traj)
                        controller.writeDMPBag(plan,name_dmp)
                    keep_record = False
                    controller.dispatchBags(obj_goal,name_ee)
                    print("DATA CONTROLLER : Bags in place")
                
                controller.resetObjectGoal()
                tmp_ob.object = -1
                tmp_ob.goal = -1
                ob.object = -1
                ob.goal = -1
                controller.removeTmpBag(name_object_blue)
                controller.removeTmpBag(name_object_green)
                controller.removeTmpBag(name_ee)
                mode = False
                motion = False

                
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


