#!/usr/bin/env python3
from ossaudiodev import control_labels
import kivy
#kivy.require('2.0.0') # replace with your current kivy version !
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import geometry_msgs.msg
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
#from hebbian_dmp.msg import ObjectGoal
from geometry_msgs.msg import Pose
from perception.msg import ObjectGoal
#from perception.msg import ListGoals
import cv2
from csv import writer
import numpy as np
import math
from kivy.app import App
from vizualisation.circular_progress_bar import *
from kivy.core.window import Window
from kivy.uix.button import Button
from kivy.properties import ListProperty, StringProperty
import pickle
from os.path import exists
import os
from os import listdir
import shutil
import http.client, urllib
from playsound import playsound

_DEFAULT_BACKGROUND_ACTIVE = (49/255, 53/255, 82/255, 0.0)
_DEFAULT_BACKGROUND_NONACTIVE = (49/255, 53/255, 82/255, 1.0)
_DEFAULT_WIDGET_ACTIVE = (1, 1, 1, 0.8)
_DEFAULT_WIDGET_NONACTIVE = (1, 1, 1, 0)
_OBJECT_BLUE = (0.110, 0.427, 0.815, 0.1)
_GREEN_LIGHT = (0.180, 0.690, 0.525, 0.9)
_RED_LIGHT = (0.721, 0.250, 0.368, 1)
_GRAY_LIGHT = (0.26,0.26,0.26,0.8)
_ACTIVE_LIGHT = (48/255,84/255,150/255,1)
_YELLOW_LIGHT = (255/255,185/255,97/255,0.8)


Window.clearcolor = (49/255, 53/255, 82/255, 0)
Window.size = (1400, 800)

class DataRecorder(object):
    def __init__(self, name_topic):
        super(DataRecorder, self).__init__()
        #rospy.init_node('DataRecorder')   
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(name_topic, Image, self.field_callback)
        self.sub_og = rospy.Subscriber("/intrinsic/new_goal", Pose, self.callbackOG)
        self.cv2_img = ""
        self.current_field = np.zeros((100,100))
        self.dim_field = [0,0]
        self.init_size = True
        self.size = 10
        self.list_peaks = []
        self.list_tmp_peaks = []
        self.list_lp = []

    def field_callback(self,msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            self.current_field = np.asarray(self.cv2_img)
        except CvBridgeError as e:
            print(e)

    def callbackOG(self,msg):
        p = [int(msg.position.x),int(msg.position.y),0.0]
        self.list_peaks.append(p)

    def getPeak(self):
        max_peak = 0
        minX = 100
        minY = 100
        maxX = 0
        maxY = 0
        X = 0
        Y = 0
        for i in range(0,self.current_field.shape[0]):
            for j in range(0,self.current_field.shape[1]):
                if self.current_field[i,j] > max_peak:
                    X = j
                    Y = i
                    max_peak = self.current_field[i,j]
        
        return X,Y,max_peak

    def getPeaks(self):
        max_peak = 0
        peak = []
        tmp_peak = []
        got_peak = False
        minX = 100
        minY = 100
        maxX = 0
        maxY = 0
        X = 0
        Y = 0
        for i in range(0,self.current_field.shape[0]):
            for j in range(0,self.current_field.shape[1]):
                tmp = [j,i]
                if self.isInlist(tmp) == False:
                    if self.current_field[i,j] > 0.01:
                        if minX == 100:
                            minX = j
                        if minY == 100:
                            minY = i
                        got_peak = True
                    else:
                        if got_peak == True:
                            maxX = j
                            maxY = i
                            px = (int)((minX + maxX)/2)
                            py = (int)((minY + maxY)/2)
                            peak = [px,py,0]
                            tmp_peak = [px,py]
                            #print("peak   ",peak)
                            #self.list_peaks.append(peak)
                            got_peak = False
        if peak and self.isInlist(peak) == False:
            self.list_peaks.append(peak)
        if tmp_peak and self.isInTMPlist(tmp_peak) == False:
            self.list_tmp_peaks.append(peak)

    def getValuePeaks(self):
        for i in range(0,len(self.list_peaks)):
            val = self.current_field[self.list_peaks[i][1],self.list_peaks[i][0]]
            self.list_peaks[i][2] = val
        #self.removeResidualPeaks()

        return self.list_peaks

    def getValueLPPeaks(self):
        self.list_lp = []
        for i in range(0,len(self.list_tmp_peaks)):
            val = self.current_field[self.list_tmp_peaks[i][1],self.list_tmp_peaks[i][0]]
            t = [self.list_tmp_peaks[i][0],self.list_tmp_peaks[i][1],val]
            self.list_lp.append(t)

        return self.list_lp

    #remove peaks induced from noise
    def removeResidualPeaks(self):
        rm = True
        i = 0
        while rm:
            i_max = len(self.list_peaks)
            if self.list_peaks[i][2] < 0.009:
                self.list_peaks.pop(i)
                i = 0
                i_max = len(self.list_peaks)
            else:
                if i+1 < i_max:
                    i += 1
                else:
                    rm = False

    def isInlist(self,peak):
        inList = False
        for i in range(0,len(self.list_peaks)):
            if self.list_peaks[i][0] <= peak[0] + 5 and self.list_peaks[i][0] >= peak[0] - 5:
                if self.list_peaks[i][1] <= peak[1] + 5 and self.list_peaks[i][1] >= peak[1] - 5:
                    inList = True

        return inList

    def isInTMPlist(self,peak):
        inList = False
        for i in range(0,len(self.list_tmp_peaks)):
            if self.list_tmp_peaks[i][0] <= peak[0] + 5 and self.list_tmp_peaks[i][0] >= peak[0] - 5:
                if self.list_tmp_peaks[i][1] <= peak[1] + 5 and self.list_tmp_peaks[i][1] >= peak[1] - 5:
                    inList = True
                    
        return inList

    def setList(self, list_p):
        self.list_tmp_peaks = []
        self.list_tmp_peaks = list_p

    def resetList(self):
        self.list_peaks = []
        
    def resetCurrentField(self):
        self.current_field = np.zeros((100,100))

    def writeDatasError(self,time):
        name = ""
        for i in range(0,len(self.list_peaks)):
            name = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/object-goal_"+str(self.list_peaks[i][0])+"_"+str(self.list_peaks[i][1])+"_ERROR.csv"
            with open(name, 'a', newline='') as f_object:
                writer_object = writer(f_object)
                row = [time,self.list_peaks[i][2]]
                writer_object.writerow(row)
                f_object.close()

    def writeDatasLP(self,time):
        name = ""
        for i in range(0,len(self.list_peaks)):
            name = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/object-goal_"+str(self.list_peaks[i][0])+"_"+str(self.list_peaks[i][1])+"_LP.csv"
            with open(name, 'a', newline='') as f_object:
                writer_object = writer(f_object)
                row = [time,self.list_peaks[i][2]]
                writer_object.writerow(row)
                f_object.close()

    def savePeaks(self,name_peaks):
        p = exists(name_peaks)
        if p:
            os.remove(name_peaks)
        filehandler = open(name_peaks, 'wb')
        pickle.dump(self.list_peaks, filehandler)

    def loadPeaks(self,name_peaks):
        print("loading Peaks")
        filehandler = open(name_peaks, 'rb') 
        self.list_peaks = pickle.load(filehandler)
        print(self.list_peaks)

class DmpListener(object):
    def __init__(self):
        super(DmpListener, self).__init__()
        self.sub = rospy.Subscriber("/motion_panda/dmp_object_goal", Pose, self.callback_dmp)
        self.goal = 0

    def callback_dmp(self,msg):
        self.goal = int(msg.position.y)

    def getDmp(self):
        return self.goal

class GoalRecorder(object):
    def __init__(self,name_topic):
        super(GoalRecorder, self).__init__()
        self.sub_goal = rospy.Subscriber(name_topic, ObjectGoal, self.callback_goal)
        self.current_goal = ObjectGoal()

    def callback_goal(self,msg):
        self.current_goal.object = msg.object
        self.current_goal.goal = msg.goal

    def rectify_object(self):
        if self.current_goal.object < 30 and self.current_goal.object > 15:
            self.current_goal.object = 24
        if self.current_goal.object < 60 and self.current_goal.object > 50:
            self.current_goal.object = 55
        if self.current_goal.object < 85 and self.current_goal.object > 75:
            self.current_goal.object = 80

    def writeCurrentGoal(self,time):
        self.rectify_object()
        name = ""
        name = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/current_goal.csv"
        with open(name, 'a', newline='') as f_object:
            writer_object = writer(f_object)
            row = [time,int(self.current_goal.goal)]
            writer_object.writerow(row)
            f_object.close()

class ControlArch(object):
    def __init__(self):
        super(ControlArch, self).__init__()
        self.pub_reset_memory = rospy.Publisher("/architecture/save_memory",Float64,queue_size=10)
        self.pub_reset_sim = rospy.Publisher("/architecture/reset_sim",Float64,queue_size=10)
        self.pub_unstuck_sim = rospy.Publisher("/architecture/unstuck",Float64,queue_size=10)
        self.pub_start_expl = rospy.Publisher("/architecture/exp",Bool,queue_size=10)
        rospy.Subscriber("/motion_panda/ee_moving", Bool, self.callbackMotion)
        self.is_moving = False
        self.start_expl = False

    def callbackMotion(self,msg):
        self.is_moving = msg.data

    def getIsMoving(self):
        return self.is_moving

    def publishUnstuck(self,val):
        uns = Float64()
        uns.data = val
        self.pub_unstuck_sim.publish(uns)

    def publishExpl(self,val):
        d = Bool()
        d.data = val
        self.pub_start_expl.publish(d)

    def saveMemory(self,val):
        res = Float64()
        res.data = val
        self.pub_reset_memory.publish(res)

    def resetArchitecture(self,val):
        res = Float64()
        res.data = val
        self.pub_reset_sim.publish(res)

class DataNodeRecorder(object):
    def __init__(self, name_topic, name_mode):
        super(DataNodeRecorder, self).__init__()
        self.sub = rospy.Subscriber(name_topic, Float64, self.node_callback)
        self.node = 0.0
        self.name_mode = name_mode 

    def node_callback(self, dat):
        self.node = dat.data

    def getNode(self):
        return self.node

    def writeValue(self,time):
        n = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/"+self.name_mode+".csv"
        with open(n, 'a', newline='') as f_object:
            writer_object = writer(f_object)
            row = [time,self.node]
            writer_object.writerow(row)
            f_object.close()


class VisualDatas(App):

    explore = ListProperty([0.26, 0.26, 0.26, 0.8])
    exploit = ListProperty([0.26, 0.26, 0.26, 0.8])
    learning_dmp = ListProperty([0.26, 0.26, 0.26, 0.8])
    retrieve_dmp = ListProperty([0.26, 0.26, 0.26, 0.8])
    start_record = ListProperty([48/255,84/255,150/255,1])
    stop_record = ListProperty([0.26, 0.26, 0.26, 0.8])
    name_record = StringProperty('Start')
    arrow = StringProperty('/home/altair/PhD/Codes/catkin_noetic/src/vizualisation/images/blank.png')

    def __init__(self, **kwargs):
        super(VisualDatas, self).__init__(**kwargs)
        self.record = False
        self.time = 0
        self.steps = 0
        rospy.init_node('DataRecorder')
        rate = rospy.Rate(50)
        self.dr_error_fwd = DataRecorder("/data_recorder/data_errors")
        self.dr_lp = DataRecorder("/data_recorder/data_lp")
        self.goal_r = GoalRecorder("/data_controller/goal_to_exploit")
        self.node_explore = DataNodeRecorder("/data_recorder/node_explore","explore")
        self.node_exploit = DataNodeRecorder("/data_recorder/node_exploit","exploit")
        self.node_learning_dmp = DataNodeRecorder("/data_recorder/hebbian","hebbian")
        self.dmp = DmpListener()
        self.control_arch = ControlArch()
        self.current_dmp = 0
        self.first = True
        self.n_exploit = True
        self.launch = True
        self.name_time = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/time.pkl"
        self.name_peaks = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/peaks.pkl"
        self.working_dir = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/"
        self.dmp_dir = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/dmp/"
        self.mode_record = "Start"
        self.index = 1
        self.name_object = "ball"
        self.experiment = "mid_lp"
        self.rec = True
        self.rst = False
        self.last_time = 0
        self.keep_exploring = False
        self.keep_exploit = False
        self.too_many = False

    def saveTime(self):
        p = exists(self.name_time)
        if p:
            os.remove(self.name_time)
        filehandler = open(self.name_time, 'wb')
        pickle.dump(self.time, filehandler)

    def loadTime(self):
        filehandler = open(self.name_time, 'rb') 
        self.time = pickle.load(filehandler)
        print(self.time)

    def notifySmartPhone(self,msg):
        conn = http.client.HTTPSConnection("api.pushover.net:443")
        conn.request("POST", "/1/messages.json",
        urllib.parse.urlencode({
            "token": "at47evsq4s32ikq1z19c113hzqszhm",
            "user": "u4duarerbk33yztxte93kevd5xx78j",
            "message": msg,
        }), { "Content-type": "application/x-www-form-urlencoded" })
        conn.getresponse()

    def notify(self,notif):
        if notif == True:
            playsound('/home/altair/PhD/Codes/ExperimentIM-LCNE/happy.wav')
        else:
            playsound('/home/altair/PhD/Codes/ExperimentIM-LCNE/wrong.wav')

    def getListTmpFiles(self,name):
        list_files = []
        for dir in os.listdir(name):
            list_files.append(dir)

        return list_files

    def removeFiles(self):
        ftr = self.getListTmpFiles(self.working_dir)
        for i in ftr:
            file = self.working_dir + i
            os.remove(file)
        #remove dmp files
        dmptr = self.getListTmpFiles(self.dmp_dir)
        for i in dmptr:
            file = self.dmp_dir + i
            os.remove(file)

    def checkNumberSkills(self):
        l_error = self.dr_error_fwd.getValuePeaks()
        size_l = len(l_error)
        if self.name_object == "cylinder" and size_l > 5:
            self.too_many = True

    def checkStuck(self):
        ee = self.control_arch.getIsMoving()
        if ee == True:
            self.last_time = self.time
        if self.time - self.last_time > 30:
            self.control_arch.publishUnstuck(1.0)
            rospy.sleep(1)
            self.control_arch.publishUnstuck(0.0)
            rospy.sleep(1)
            self.last_time = self.time


    def checkSkills(self):
        end_sim = True
        l_error = self.dr_error_fwd.getValuePeaks()
        l_lp = self.dr_lp.getValuePeaks()
        size_l = len(l_error)
        if size_l < 1:
            end_sim = False
        for i in range(0,size_l):
            err = math.ceil(l_error[i][2]*100)/100
            lp = math.ceil(l_lp[i][2]*100)/100
            err = err * 100
            lp = lp * 100
            if err >= 4 or lp >= 9:
                end_sim = False
        if self.time < 120:
            end_sim = False

        return end_sim

    def checkSimulation(self):
        end = self.checkSkills()
        if end:
            self.set_record()
            self.control_arch.resetArchitecture(1.0)
            rospy.sleep(1.0)
            #self.control_arch.saveMemory(1.0)
            #print("RESET CALLED : SAVING DFT MEMORY")
            #rospy.sleep(1)
            #self.control_arch.saveMemory(0.0)
            self.saveTime()
            print("RESET CALLED : RESET SKILLS LIST")
            self.dr_error_fwd.resetList()
            self.dr_lp.resetList()
            self.rst = True
            print("RESET CALLED : SAVING ALL DATAS")
            self.copy_all_datas(self.name_object,self.experiment,self.index)
            self.removeFiles()
            print("RESET CALLED : RESET DFT ARCHITECTURE")
            #msg = "Experiment "+ self.name_object + " " + str(self.index) + " over"
            #self.notifySmartPhone(msg)
            self.notify(True)
            print("START OF NEW EXPERIMENT")
            self.set_record()
            self.time = 0
            self.last_time = 0
            self.index += 1
            self.keep_exploit = False
        if self.too_many or self.keep_exploring:
            #msg = "Experiment "+ self.name_object + " " + str(self.index) + " failed"
            self.notify(False)
            self.too_many = False
            self.keep_exploring = False
            self.index = self.index - 1

    def startExploration(self):
        self.control_arch.publishExpl(True)
        self.control_arch.resetArchitecture(0.0)
        rospy.sleep(1)
        self.control_arch.publishExpl(False)


    def copy_all_datas(self,name_object,type_exp,index):
        list_files = self.getListTmpFiles(self.working_dir)
        for i in list_files:
            source = self.working_dir + i
            new_dest = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/" + name_object + "/" + type_exp + "/" + str(index) + "/"
            newPath = shutil.copy(source, new_dest)

    def copy_datas_explore(self,name_object,type_exp,index):
        list_files = self.getListTmpFiles(self.working_dir)
        for i in list_files:
            source = self.working_dir + i
            new_dest = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/" + name_object + "/" + type_exp + "/" + str(index) + "/mid/"
            newPath = shutil.copy(source, new_dest)
        list_dmp = self.getListTmpFiles(self.dmp_dir)
        for i in list_dmp:
            src_dmp = self.dmp_dir + i
            dest_dmp = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/" + name_object + "/" + type_exp + "/" + str(index) + "/dmp/"
            newPath = shutil.copy(src_dmp, dest_dmp)


    def set_record(self):
        if self.mode_record == "Start":
            self.name_record = "Stop"
            self.mode_record = "Stop"
            self.start_record = _RED_LIGHT
            self.record = True
            self.first = False
        else:
            self.name_record = "Start"
            self.mode_record = "Start"
            self.start_record = _GREEN_LIGHT
            self.saveTime()
            self.dr_error_fwd.savePeaks(self.name_peaks)
            self.record = False

    def load_datas(self):
        self.loadTime()
        self.dr_error_fwd.loadPeaks(self.name_peaks)
        self.dr_lp.loadPeaks(self.name_peaks)

    def update_image(self,inp):
        name = "/home/altair/PhD/Codes/catkin_noetic/src/vizualisation/images/"+str(inp)+".png"
        self.arrow = name

    def update_events(self, dt):
        if self.node_explore.getNode() > 0.8:
            self.explore = _GREEN_LIGHT
            self.retrieve_dmp = _GRAY_LIGHT
            self.n_exploit = True
            self.update_image("blank")
            if self.keep_exploit == True:
                self.keep_exploring = True
        else:
            self.explore = _GRAY_LIGHT
        if self.node_exploit.getNode() > 0.8:
            self.exploit = _GREEN_LIGHT
            self.retrieve_dmp = _YELLOW_LIGHT
            self.keep_exploit = True
            """if self.n_exploit == True:
                self.saveTime()
                self.dr_error_fwd.savePeaks(self.name_peaks)
                self.copy_datas_explore(self.name_object,self.experiment,self.index)
                self.n_exploit = False"""
        else:
            self.exploit = _GRAY_LIGHT
        if self.node_learning_dmp.getNode() > 0.5 and self.node_learning_dmp.getNode() < 1.5:
            self.learning_dmp = _YELLOW_LIGHT
        else:
            self.learning_dmp = _GRAY_LIGHT


    # Update the progress with datas coming from neural fields
    def update_gauges(self, dt):
        #self.checkNumberSkills()
        self.checkSimulation()
        self.checkStuck()
        tmp_dmp =  self.dmp.getDmp()
        if tmp_dmp > 0 and tmp_dmp != self.current_dmp:
            self.update_image(int(tmp_dmp))
            self.current_dmp = tmp_dmp
        list_error_fwd = self.dr_error_fwd.getValuePeaks()
        list_lp = self.dr_lp.getValuePeaks()
        size_l = len(list_error_fwd)
        if self.record == True:
            if self.first == True:
                self.time = 0
                self.first = False
            if self.steps == 10:
                self.dr_error_fwd.writeDatasError(int(self.time))
                self.dr_lp.writeDatasLP(int(self.time))
                self.goal_r.writeCurrentGoal(int(self.time))
                #self.node_explore.writeValue(int(self.time))
                #self.node_exploit.writeValue(int(self.time))
                self.steps = 0
            self.steps += 1
        for i in range(0,size_l):
            err = math.ceil(list_error_fwd[i][2]*100)/100
            lp = math.ceil(list_lp[i][2]*100)/100
            if err > 1.0:
                err = 1.0
            if lp > 1.0:
                lp = 1.0
            self.root.children[0].children[i].value_normalized_error = err
            self.root.children[0].children[i].value_normalized = lp
            self.root.children[0].children[i].value_error_string = str(err*100)
            self.root.children[0].children[i].value_lp = str(lp*100)
            self.root.children[0].children[i].value_normalized_goal = str(int(list_error_fwd[i][1]))
            if list_error_fwd[i][0] < 30:
                self.root.children[0].children[i].value_object = 0
            if list_error_fwd[i][0] > 30 and list_error_fwd[i][0] < 60:
                self.root.children[0].children[i].value_object = 1
            if list_error_fwd[i][0] > 60:
                self.root.children[0].children[i].value_object = 2
            self.root.children[0].children[i].background_non_active = _DEFAULT_BACKGROUND_ACTIVE
            self.root.children[0].children[i].background_active = _DEFAULT_WIDGET_NONACTIVE
        if self.rst == True:
            for j in range(0,2):
                for i in range(0,len(self.root.children[0].children)):
                    self.root.children[0].children[i].value_normalized_error = 0
                    self.root.children[0].children[i].value_normalized = 0
                    self.root.children[0].children[i].value_error_string = 0
                    self.root.children[0].children[i].value_lp = 0
                    self.root.children[0].children[i].value_normalized_goal = str(int(0))
                    self.root.children[0].children[i].background_non_active = _DEFAULT_BACKGROUND_NONACTIVE
                    self.root.children[0].children[i].background_active = _DEFAULT_BACKGROUND_NONACTIVE
                    #self.root.children[0].children[i].background_colour = _DEFAULT_BACKGROUND_NONACTIVE
            self.startExploration()
            self.rst = False
        if self.record == True:
            self.time += 0.1

    # Simple layout for easy example
    def build(self):
        
        self.container = Builder.load_string('''
#:import Label kivy.core.text.Label           
#:set _label Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lp Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goal Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labelone Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpone Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goalone Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labeltwo Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lptwo Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goaltwo Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labelthree Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpthree Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goalthree Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labelfour Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpfour Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goalfour Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labelfive Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpfive Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goalfive Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labelsix Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpsix Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goalsix Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labelseven Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpseven Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goalseven Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labeleight Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpeight Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goaleight Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labelnine Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpnine Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goalnine Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labelten Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpten Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goalten Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _labeleleven Label(text="ERROR {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_lpeleven Label(text="LP {}%", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
#:set _label_goaleleven Label(text="Goal : {}", font_size=16, color=(0.933,0.902,0.807,1), halign="center")
BoxLayout:
    id: bl
    orientation: 'horizontal'
    BoxLayout:
        orientation: 'vertical'
        size: 400, 800
        size_hint: (None,None)
        BoxLayout:
            orientation: 'horizontal'
            size: 400, 100
            size_hint: (None,None)
            Label:
                text_size: self.size
                size: self.texture_size
                halign: 'center'
                valign: 'middle'
                font_size: 32
                text: "explore"
                canvas.before:
                    Color:
                        rgb: app.explore
                    RoundedRectangle:
                        size: self.size
                        pos: self.pos
                        radius: [55]
            Label:
                text_size: self.size
                size: self.texture_size
                halign: 'center'
                valign: 'middle'
                font_size: 32
                text: "exploit"
                canvas.before:
                    Color:
                        rgba: app.exploit
                    RoundedRectangle:
                        size: self.size
                        pos: self.pos
                        radius: [55]
        Label:
            text_size: self.size
            size: self.texture_size
            halign: 'center'
            valign: 'middle'
            font_size: 32
            text: "Learning DMP"
            canvas.before:
                Color:
                    rgb: app.learning_dmp
                RoundedRectangle:
                    size: 400,150
                    pos: 0, 515
                    radius: [55]
        Label:
            text_size: self.size
            size: self.texture_size
            halign: 'center'
            valign: 'middle'
            #pos: 0, -100
            font_size: 32
            text: "Retrieve DMP"
            canvas.before:
                Color:
                    rgb: app.retrieve_dmp
                RoundedRectangle:
                    size: 400,150
                    pos: 0, 300
                    radius: [55]
        FloatLayout:
            pos: 100,100
            Image:
                size_hint: None, None
                size: 200, 200
                pos: 100,100
                source: app.arrow
            
        BoxLayout:
            orientation: 'horizontal'
            size: 400, 70
            size_hint: (None,None)
            Button:
                #text: "Start record"
                text: app.name_record
                font_size: 24
                size: 50, 50
                pos: 10, 10
                background_color: app.start_record
                #background_normal: ''
                on_press: app.set_record()
            Button:
                text: "Load Datas"
                font_size: 24
                size: 50, 50
                pos: 10, 10
                background_color: app.stop_record
                #background_normal: ''
                on_press: app.load_datas()
    GridLayout:
        id: gl
        cols: 4
        rows: 3
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _label
            label_lp: _label_lp
            label_goal: _label_goal
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labelone
            label_lp: _label_lpone
            label_goal: _label_goalone
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labeltwo
            label_lp: _label_lptwo
            label_goal: _label_goaltwo
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labelthree
            label_lp: _label_lpthree
            label_goal: _label_goalthree
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labelfour
            label_lp: _label_lpfour
            label_goal: _label_goalfour
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labelfive
            label_lp: _label_lpfive
            label_goal: _label_goalfive
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labelsix
            label_lp: _label_lpsix
            label_goal: _label_goalsix
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labelseven
            label_lp: _label_lpseven
            label_goal: _label_goalseven
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labeleight
            label_lp: _label_lpeight
            label_goal: _label_goaleight
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labelnine
            label_lp: _label_lpnine
            label_goal: _label_goalnine
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labelten
            label_lp: _label_lpten
            label_goal: _label_goalten
        CircularProgressBar:
            pos: 100, 100
            thickness: 5
            widget_size: 150
            label_error_string: _labeleleven
            label_lp: _label_lpeleven
            label_goal: _label_goaleleven''')
        # Animate the progress bar
        Clock.schedule_interval(self.update_gauges, 0.1)
        Clock.schedule_interval(self.update_events, 0.1)
        return self.container


if __name__ == '__main__':
    #rospy.init_node('DataRecorder')
    #rate = rospy.Rate(50)
    VisualDatas().run()
