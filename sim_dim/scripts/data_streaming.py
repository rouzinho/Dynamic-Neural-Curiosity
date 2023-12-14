#!/usr/bin/env python3
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
from perception.msg import ObjectController


class DataController(object):
    def __init__(self):
        super(DataController, self).__init__()
        rospy.init_node('controller', anonymous=True)
        self.ee_pose = geometry_msgs.msg.Pose()
        self.blue_pos = geometry_msgs.msg.Pose()
        self.yellow_pos = geometry_msgs.msg.Pose()
        self.red_pos = geometry_msgs.msg.Pose()
        self.pub_nn_controller = rospy.Publisher('/intrinsic/controller', ObjectController, latch=True, queue_size=10)
        rospy.Subscriber("/motion_panda/ee_moving", Bool, self.callbackMotion)
        rospy.Subscriber("/motion_panda/end_action", Bool, self.callbackEndAction)
        rospy.Subscriber("/proprioception_ee", Pose, self.callbackProprioception)
        rospy.Subscriber("/cube_position", Pose, self.callbackBluePosition)
        rospy.Subscriber("/cylinder_position", Pose, self.callbackYellowPosition)
        rospy.Subscriber("/ball_position", Pose, self.callbackRedPosition)
        rospy.Subscriber("/perception/list_goals", ListGoals, self.callbackPerception)
        rospy.Subscriber("/data_recorder/node_explore", Float64, self.callbackExplore)
        rospy.Subscriber("/data_recorder/node_exploit", Float64, self.callbackExploit)
        rospy.Subscriber("/data_controller/goal_to_exploit", ObjectGoal, self.callbackExploitGoal)
        rospy.Subscriber("/data_controller/learning", ObjectGoal, self.callbackLearningDMP)
        rospy.Subscriber("/architecture/reset_sim", Float64, self.callbackReset)
        self.object_controller = ObjectController()
        self.is_moving = False
        self.end_action = False
        self.dims = 3 
        self.dt = 1.0
        self.K_gain = 100              
        self.D_gain = 2.0 * np.sqrt(self.K_gain)      
        self.num_bases = 4
        self.list_goals = ListGoals()
        self.list_samples = []
        self.goal_to_exploit = ObjectGoal()
        self.goal_to_exploit.object = -1
        self.goal_to_exploit.goal = -1
        self.learning_dmp = ObjectGoal()
        self.learning_dmp.object = -1
        self.learning_dmp.goal = -1
        self.explore = 0.0
        self.exploit = 0.0
        self.reset = False

    def callbackMotion(self,msg):
        self.is_moving = msg.data

    def callbackEndAction(self,msg):
        self.end_action = msg.data

    def callbackProprioception(self,ee):
        self.ee_pose.position.x = ee.position.x
        self.ee_pose.position.y = ee.position.y
        self.ee_pose.position.z = ee.position.z
        
    def callbackBluePosition(self,cube):
        self.blue_pos.position.x = cube.position.x
        self.blue_pos.position.y = cube.position.y
        self.blue_pos.position.z = cube.position.z

    def callbackYellowPosition(self,cylinder):
        self.yellow_pos.position.x = cylinder.position.x
        self.yellow_pos.position.y = cylinder.position.y
        self.yellow_pos.position.z = cylinder.position.z

    def callbackRedPosition(self,r):
        self.red_pos.position.x = r.position.x
        self.red_pos.position.y = r.position.y
        self.red_pos.position.z = r.position.z

    def callbackPerception(self,lg):
        self.list_goals.obj_goals = []
        for i in range(0,len(lg.obj_goals)):
            tmp_og = ObjectGoal()
            tmp_og.object = lg.obj_goals[i].object
            tmp_og.goal = lg.obj_goals[i].goal
            self.list_goals.obj_goals.append(tmp_og)

    def callbackExplore(self,msg):
        self.explore = msg.data

    def callbackExploit(self,msg):
        self.exploit = msg.data

    def callbackExploitGoal(self,obg):
        self.goal_to_exploit.object = obg.object
        self.goal_to_exploit.goal = obg.goal

    def callbackLearningDMP(self,obg):
        self.learning_dmp.object = obg.object
        self.learning_dmp.goal = obg.goal

    def callbackReset(self,msg):
        tmp = msg.data
        if tmp > 0.5:
            self.reset = True
        else:
            self.reset = False

    def getReset(self):
        return self.reset

    def publishDataController(self):
        self.pub_nn_controller.publish(self.object_controller)

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

    #write yellow object motion in a bag
    def writeBagYellowObjectPos(self,n):
        name = n
        exist = path.exists(name)
        opening = ""
        if(exist == True):
            opening = "a"
        else:
            opening = "w"
        bag = rosbag.Bag(name, opening)
        try:
            bag.write("object_pos",self.yellow_pos)
        finally:
            bag.close()

    def writeBagRedObjectPos(self,n):
        name = n
        exist = path.exists(name)
        opening = ""
        if(exist == True):
            opening = "a"
        else:
            opening = "w"
        bag = rosbag.Bag(name, opening)
        try:
            bag.write("object_pos",self.red_pos)
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

        return obg, inlist

    #create  first and last position of object + goal position of EE and dispatch all of them in appropriate repositories for machine learning
    def dispatchBags(self,obg,name_ee,val):
        obj = ""
        objf = 0.0
        if int(obg.object) <= 60 and  int(obg.object) >= 49 :
            obj = "55"
            objf = 55.0
        if int(obg.object) <= 30 and  int(obg.object) >= 18 :
            obj = "24"
            objf = 24.0
        if int(obg.object) <= 85 and  int(obg.object) >= 75 :
            obj = "80"
            objf = 80.0
        name_object = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_"+ obj +".bag"
        self.genInitialObjectPosition(name_object)
        self.genLastObjectPosition(name_object)
        self.genLastEEPosition(name_ee)
        self.object_controller.object_model.object = objf
        self.object_controller.object_model.goal = obg.goal
        self.object_controller.object_model.value = val
        self.publishDataController()
        
        
    def genInitialObjectPosition(self,n):
        name_open = n
        bag_open = rosbag.Bag(name_open)
        first = True
        for topic, msg, t in bag_open.read_messages(topics=['object_pos']):
            if first == True:
                self.object_controller.object_fpos.position.x = msg.position.x
                self.object_controller.object_fpos.position.y = msg.position.y
                first = False
        bag_open.close()

    def genLastEEPosition(self,n):
        name_open = n
        bag_open = rosbag.Bag(name_open)
        for topic, msg, t in bag_open.read_messages(topics=['ee_pos']):
            self.object_controller.ee_last_pos.position.x = msg.position.x
            self.object_controller.ee_last_pos.position.y = msg.position.y
        bag_open.close()

    def genLastObjectPosition(self,n):
        name_open = n
        bag_open = rosbag.Bag(name_open)
        for topic, msg, t in bag_open.read_messages(topics=['object_pos']):
            self.object_controller.object_lpos.position.x = msg.position.x
            self.object_controller.object_lpos.position.y = msg.position.y
        bag_open.close()
        

    def removeTmpBag(self,n):
        if path.exists(n):
            os.remove(n)

    def getMoving(self):
        return self.is_moving

    def getEndAction(self):
        return self.end_action

    def getLearningDMP(self):
        return self.learning_dmp

    def getPerception(self):
        return self.list_goals

    def isPerceived(self):
        percept = False
        for i in range(0,len(self.list_goals.obj_goals)):
            if self.list_goals.obj_goals[i].object > 0 and self.list_goals.obj_goals[i].goal > 0:
                percept = True

        return percept

    def resetPerception(self):
        self.list_goals = ListGoals()

    def resetSamples(self):
        self.list_samples = []

    def getExplore(self):
        return self.explore

    #def resetGoalToExploit(self):
    #    self.goal_to_exploit.object = -1
    #    self.goal_to_exploit.goal = -1

    def resetLearningDMP(self):
        self.learning_dmp.object = -1
        self.learning_dmp.goal = -1    

    def getGoalToExploit(self):
        return self.goal_to_exploit

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
        name_object_blue = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_24.bag"
        name_object_yellow = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_55.bag"
        name_object_red = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_80.bag"
        name_ee = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/motion_EE.bag"
        motion = False
        got_perception = False
        mode = False
        rm_bags = True
        set_mode = 0
        controller = DataController()
        list_g = ListGoals()
        tmp_learning = ObjectGoal()
        tmp_learning.object = -1
        tmp_learning.goal = -1
        learning = False
        obj_goal = ObjectGoal()
        obj_goal.object = -1
        obj_goal.goal = -1
        og_exploit = ObjectGoal()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rst = controller.getReset()
            if rst == True:
                controller.resetSamples()
            #if EE is performing an action
            if controller.getMoving() == True:
                #record the action, the object motion and check if the goal angle is new
                motion = True
                if controller.getExplore() > 0.8:
                    set_mode = 1
                else:
                    set_mode = 2
                controller.writeBagEE(name_ee)
                controller.writeBagBlueObjectPos(name_object_blue)
                controller.writeBagYellowObjectPos(name_object_yellow) 
                controller.writeBagRedObjectPos(name_object_red)
                rm_bags = True
            if controller.getMoving() == False and motion == True:
                tmp_learning = controller.getLearningDMP()
                if tmp_learning.object > 0:
                    learning = True
                list_g = controller.getPerception()
                got_perception = controller.isPerceived()
            #condition in order to accelerate the processing of datas
            # basically if learning (meaning creating a new dmp) or it's exploiting and we have the perceptions
            # if it's exploring and no learning OR it's exploiting and there is no change in the environment THEN we don't do anything
            if learning or (set_mode == 2 and got_perception):
                if learning:
                    #detector is resilient but there might be a slight difference
                    obj_goal, in_list = controller.renameFreshBag(tmp_learning)
                    name_object = str(int(obj_goal.object))
                    name_goal = str(int(obj_goal.goal))
                    print("DATA CONTROLLER : modified obj ",obj_goal)
                    if not in_list:
                        if set_mode == 1 and obj_goal.object > 0:
                            print("DATA CONTROLLER : write DMP")
                            name_dmp = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/dmp/dmp_" + name_object + "_" + name_goal + ".bag"
                            traj = controller.formDatasEE(name_ee)
                            plan = controller.makeLFDRequest(traj)
                            controller.writeDMPBag(plan,name_dmp)
                            controller.dispatchBags(obj_goal,name_ee,1.0)
                    learning = False
                if got_perception and set_mode == 2:
                    og_exploit = controller.getGoalToExploit()
                    print(og_exploit)
                    print("Exploit and changes in environment")
                    obj_goal, in_list = controller.renameFreshBag(og_exploit)
                    print(obj_goal)
                    print(list_g)
                    val = 0.0
                    for i in range(0,len(list_g.obj_goals)):
                        if list_g.obj_goals[i].object > 0:
                            val = list_g.obj_goals[i].goal
                    if obj_goal.object > 0:
                        controller.dispatchBags(obj_goal,name_ee,val)
                    got_perception = False
                motion = False
            if controller.getEndAction() and rm_bags:
                print("removing bags")
                obj_goal.object = -1
                obj_goal.goal = -1
                list_g.obj_goals = []
                tmp_learning.object = -1
                tmp_learning.goal = -1
                controller.removeTmpBag(name_object_blue)
                controller.removeTmpBag(name_object_yellow)
                controller.removeTmpBag(name_object_red)
                controller.removeTmpBag(name_ee)
                controller.resetPerception()
                got_perception = False
                motion = False
                rm_bags = False
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


