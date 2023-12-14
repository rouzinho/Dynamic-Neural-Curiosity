#! /usr/bin/python

# rospy for the subscriber
import re
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import geometry_msgs.msg
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
#from hebbian_dmp.msg import ObjectGoal
from geometry_msgs.msg import Pose
from perception.msg import ObjectGoal
from perception.msg import ListGoals
import cv2
from numpy import save
# load numpy array from npy file
from numpy import load
from os.path import exists
import os
import numpy as np
import pickle

class HebbServer(object):
    def __init__(self):
        super(HebbServer, self).__init__()
        rospy.init_node('hebbserver')   
        self.dmp = []
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/hebbian_server/goal", Image, self.field_callback)
        self.sub_reward = rospy.Subscriber("/hebbian_server/reward", Float64, self.reward_callback)
        self.sub_active_dmp = rospy.Subscriber("/hebbian_server/dmp", Float64, self.dmp_callback)
        self.sub_reset = rospy.Subscriber("/architecture/reset_sim", Float64, self.callbackReset)
        self.pub_data = rospy.Publisher("/data_recorder/hebbian",Float64,queue_size=10)
        self.pub_learning = rospy.Publisher("/data_controller/learning",ObjectGoal,queue_size=10)
        self.pub_activate_dmp = rospy.Publisher("/motion_panda/dmp_object_goal",Pose,queue_size=10)
        self.cv2_img = ""
        self.current_field = np.zeros((100,100))
        self.dmp_neurons = np.zeros((1,20))
        self.weights = np.zeros((100,100,20))
        self.dim_field = [0,0]
        self.weights_init = False
        self.reward = 0
        self.activate_dmp = 0
        self.current_objgoal = ObjectGoal()
        self.list_dmp_ogoals = ListGoals()
        self.init_size = True
        self.size = 10
        self.weights_name = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/hebbian_weights.npy"
        self.name_dmps = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/dmps.pkl"
        self.loading = rospy.get_param("/load_datas")
        self.reset = False

    def field_callback(self,msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.current_field = np.asarray(self.cv2_img)
        except CvBridgeError as e:
            print(e)

    def reward_callback(self,r):
        self.reward = r.data

    def dmp_callback(self,msg):
        self.activate_dmp = msg.data

    def callbackReset(self,msg):
        tmp = msg.data
        if tmp > 0.5:
            self.reset = True
        else:
            self.reset = False

    def publishDMPActivation(self,msg):
        self.pub_activate_dmp.publish(msg)

    def publishLearning(self,msg):
        self.pub_learning.publish(msg)

    def publishDataRecorder(self,msg):
        self.pub_data.publish(msg)

    def init_weights(self):
        self.weights = np.zeros((np.size(self.dmp_neurons,1),100,100))

    def getObjectGoal(self):
        tmp_og = ObjectGoal()
        peak = False
        minX = 100
        minY = 100
        maxX = 0
        maxY = 0
        for i in range(0,self.current_field.shape[0]):
            for j in range(0,self.current_field.shape[1]):
                if self.current_field[i,j] > 0.8:
                    peak = True
                    if j < minX:
                        minX = j
                    if i < minY:
                        minY = i
                    if j > maxX:
                        maxX = j
                    if i > maxY:
                        maxY = i
                tmp_og.object = (minX+maxX)/2
                tmp_og.goal = (minY+maxY)/2
        
        return tmp_og,peak

    def getReward(self):
        return self.reward

    def getReset(self):
        return self.reset

    def getDmpActivation(self):
        return self.activate_dmp

    def printWeights(self):
        print(self.weights)

    def printDmpList(self):
        print(self.list_dmp_ogoals.obj_goals)

    def addDmpList(self,dmp):
        self.list_dmp_ogoals.obj_goals.append(dmp)

    def getDmpElement(self,ind):
        return self.list_dmp_ogoals.obj_goals[ind]

    def hebbianLearning(self): #lupdate weights when there is a incoming field with reward
        position = len(self.list_dmp_ogoals.obj_goals)-1
        self.weights[position] = np.ones((100,100))
        self.weights[position] = np.multiply(self.weights[position],self.current_field)
        #self.dmp_neurons[0,position] = 1.0
        #for i in range(0,self.weights.shape[0]):
        #    self.weights[i] = self.weights[i] + (self.dmp_neurons[0,i] * self.current_field)

    def hebbianActivation(self): #tailored for 100x100 matrix - retrieve  the index of a dmp for the list_dmpgoals from a neural activation (uses weights) 
        ind = -1
        for i in range(0,self.weights.shape[0]):
            res = 0
            mat = np.multiply(self.weights[i],self.current_field)
            res = np.sum(mat)
            #print("i : ",i)
            #print(res)
            if res >= 25:
                ind = i
                

        return ind

    def isInList(self,tmp_og):
        inlist = False
        for i in range(0,len(self.list_dmp_ogoals.obj_goals)):
            if self.list_dmp_ogoals.obj_goals[i].object <= tmp_og.object+1 and self.list_dmp_ogoals.obj_goals[i].object >= tmp_og.object-1: #tolerance for perception fluctuation
                if self.list_dmp_ogoals.obj_goals[i].goal <= tmp_og.goal+1 and self.list_dmp_ogoals.obj_goals[i].goal >= tmp_og.goal-1:
                    inlist = True
        
        return inlist

    def normalizeField(self):
        for i in range(0,self.current_field.shape[0]):
            for j in range(0,self.current_field.shape[1]):
                if self.current_field[i,j] < 0.05:
                    self.current_field[i,j] = 0.0
        
    def resetCurrentField(self):
        self.current_field = np.zeros((100,100))

    def resetListGoals(self):
        self.list_dmp_ogoals = ListGoals()

    def saveWeights(self):
        w_ex = exists(self.weights_name)
        dmp_ex = exists(self.name_dmps)
        if w_ex:
            os.remove(self.weights_name)
        if dmp_ex:
            os.remove(self.name_dmps)
        
        save(self.weights_name,self.weights)
        filehandler = open(self.name_dmps, 'wb')
        pickle.dump(self.list_dmp_ogoals, filehandler)
        
        
    def loadWeights(self):
        if self.loading == True:
            print("loading Hebbian Weights")
            self.weights = load(self.weights_name)
            filehandler = open(self.name_dmps, 'rb') 
            self.list_dmp_ogoals = pickle.load(filehandler)
            print(self.list_dmp_ogoals)
        else:
            print("Initialize Random Weights")
            self.init_weights()

if __name__ == "__main__":
    hebb_srv = HebbServer()
    rate = rospy.Rate(30)
    data_r = Float64()
    new_sample = True
    new_activation = True
    old_dmp = ObjectGoal()
    hebb_srv.loadWeights()
    while not rospy.is_shutdown():
        if hebb_srv.getReset() == True:
            hebb_srv.init_weights()
            hebb_srv.resetListGoals()
            print("HEBBIAN MODULE : RESET WEIGHTS")
        if hebb_srv.getReward() > 0.8 : #if it receives the signal for learning coming from the NF - avoiding to constantly publish empty matrix
            tmp_og, peak = hebb_srv.getObjectGoal()
            in_list = hebb_srv.isInList(tmp_og)
            if peak and not in_list:
                print("HEBBIAN MODULE : LEARNING")
                data_r.data = 1.0
                hebb_srv.publishDataRecorder(data_r)
                hebb_srv.addDmpList(tmp_og)
                hebb_srv.normalizeField()
                hebb_srv.hebbianLearning()
                hebb_srv.publishLearning(tmp_og)
                hebb_srv.resetCurrentField()
                hebb_srv.printDmpList()
                hebb_srv.saveWeights()
            else:
                data_r.data = 0.0
                hebb_srv.publishDataRecorder(data_r)
        if hebb_srv.getDmpActivation() > 0.8 :
            hebb_srv.normalizeField()
            index = hebb_srv.hebbianActivation()
            #new_activation = False
            if index != -1:
                data_r.data = 2.0
                hebb_srv.publishDataRecorder(data_r)
                tmp = hebb_srv.getDmpElement(index)
                if abs(old_dmp.object - tmp.object) > 1 or abs(old_dmp.goal - tmp.goal) > 1:
                    print("HEBBIAN MODULE activating DMP")
                    print(tmp)
                    old_dmp.object = tmp.object
                    old_dmp.goal = tmp.goal
                tmp_pose = geometry_msgs.msg.Pose()
                tmp_pose.position.x = tmp.object
                tmp_pose.position.y = tmp.goal
                #print(tmp)
                hebb_srv.publishDMPActivation(tmp_pose)
            else:
                data_r.data = 0.0
                hebb_srv.publishDataRecorder(data_r)
            hebb_srv.resetCurrentField()
        data_r.data = 0.0
        #if hebb_srv.getReward() < 0.8:
        #    new_sample = True
        #if hebb_srv.getDmpActivation() < 0.8:
        #    new_activation = True
        rate.sleep()

