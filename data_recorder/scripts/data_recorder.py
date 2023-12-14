#! /usr/bin/python

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from perception.msg import ObjectGoal
# OpenCV2 for saving an image
import cv2
from csv import writer

import numpy as np

class DataRecorder(object):
    def __init__(self, name_topic):
        super(DataRecorder, self).__init__()
        #rospy.init_node('DataRecorder')   
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(name_topic, Image, self.field_callback)
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
        if tmp_peak and self.isInlist(tmp_peak) == False:
            self.list_tmp_peaks.append(peak)
        
        return self.list_tmp_peaks

    def getValuePeaks(self):
        for i in range(0,len(self.list_peaks)):
            val = self.current_field[self.list_peaks[i][1],self.list_peaks[i][0]]
            self.list_peaks[i][2] = val

        return self.list_peaks

    def getValueLPPeaks(self):
        for i in range(0,len(self.list_tmp_peaks)):
            val = self.current_field[self.list_tmp_peaks[i][1],self.list_tmp_peaks[i][0]]
            t = [self.list_tmp_peaks[i][1],self.list_tmp_peaks[i][0],val]
            self.list_lp.append(t)

        return self.list_lp

    def isInlist(self,peak):
        inList = False
        for i in range(0,len(self.list_peaks)):
            if self.list_peaks[i][0] <= peak[0] + 5 and self.list_peaks[i][0] >= peak[0] - 5:
                if self.list_peaks[i][1] <= peak[1] + 5 and self.list_peaks[i][1] >= peak[1] - 5:
                    inList = True

        return inList

    def setList(self, list_p):
        self.list_tmp_peaks = list_p
        
    def resetCurrentField(self):
        self.current_field = np.zeros((100,100))

    def writeDatas(self,obj,time,value):
        inl, ob = self.isInList(obj)
        name = ""
        row = [time,value]
        if inl == True:
            name = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/recordings/"+str(ob.object)+"_"+str(ob.goal)+".csv"
        else:
            name = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/recordings/"+str(obj.object)+"_"+str(obj.goal)+".csv"
            self.list_peaks.append(obj)
        with open(name, 'a', newline='') as f_object:
            writer_object = writer(f_object)
            writer_object.writerow(row)
            f_object.close()

if __name__ == "__main__":
    rospy.init_node('DataRecorder')
    rate = rospy.Rate(50)
    dr_error_fwd = DataRecorder("/data_errors")
    dr_lp = DataRecorder("/data_lp")
    time = 0
    while not rospy.is_shutdown():
        l_peaks = dr_error_fwd.getPeaks()
        #list_error_fwd = self.dr_error_fwd.getValuePeaks()
        #self.dr_lp.setList(l_peaks)
        #list_lp = self.dr_lp.getValueLPPeaks()
        #print(t)
        print("                     ")
        print("-----------------")
        print("                 ")
        rate.sleep()

