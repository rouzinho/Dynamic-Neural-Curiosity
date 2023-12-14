#!/usr/bin/env python3
import re
import sys
import copy

#from torch._C import T
import rospy
import rosbag
from std_msgs.msg import Float64
import time
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
import rospy
import numpy as np
from math import pi
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import os.path
from os import listdir
from os import path
import torch
from torch.utils.data import TensorDataset, DataLoader
import torch.nn as nn
from sklearn.preprocessing import MinMaxScaler
from torch.autograd import Variable
from scipy.ndimage import gaussian_filter1d
from perception.msg import ObjectGoal
#from visdom import Visdom
import random
import glob
from pathlib import Path

# torch.cuda.is_available() checks and returns a Boolean True if a GPU is available, else it'll return False
is_cuda = torch.cuda.is_available()

# If we have a GPU available, we'll set our device to GPU. We'll use this device variable later in our code.
if is_cuda:
    device = torch.device("cuda")
    print("GPU is available")
else:
    device = torch.device("cpu")
    print("GPU not available, CPU used")


class MultiLayer(nn.Module):
    def __init__(self,input_layer,middle_layer,output_layer):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(input_layer, middle_layer),
            nn.Tanh(),
            nn.Linear(middle_layer, middle_layer),
            nn.Tanh(),
            nn.Linear(middle_layer, output_layer)
        )
        
    def forward(self, x):
        return self.layers(x)
    
class Skill(object):
    def __init__(self,input_layer_iv,middle_layer_iv,output_layer_iv,input_layer_fw,middle_layer_fw,output_layer_fw,mem_size,obj):
        self.input_layer_iv = input_layer_iv
        self.middle_layer_iv = middle_layer_iv
        self.output_layer_iv = output_layer_iv
        self.input_layer_fw = input_layer_fw
        self.middle_layer_fw = middle_layer_fw
        self.output_layer_fw = output_layer_fw
        self.inverse_model = MultiLayer(self.input_layer_iv,self.middle_layer_iv,self.output_layer_iv)
        self.inverse_model.to(device)
        self.forward_model = MultiLayer(self.input_layer_fw,self.middle_layer_fw,self.output_layer_fw)
        self.forward_model.to(device)
        self.memory_size = mem_size
        self.mod_object = int(obj)

        rospy.init_node('skills', anonymous=True)
        self.pub_update_lp = rospy.Publisher('/intrinsic/learning_progress', Float64, latch=True, queue_size=10)
        self.pub_timing = rospy.Publisher('/intrinsic/updating_lp', Float64, latch=True, queue_size=10)
        self.pub_end = rospy.Publisher('/intrinsic/end_action', Bool, queue_size=10)
        rate = rospy.Rate(100)
        self.list_goals = []
        self.inputs_both_model = []
        self.memory = []
        self.error = Float64()

    def getModObject(self):
        return self.mod_object

    def getListGoals(self):
        for dir in os.listdir("/home/altair/PhD/Codes/catkin_motion/rosbags/experiment/goals/"):
            self.list_goals.append(dir)

    def pubUpdateLP(self,error):
        e = Float64(error)
        self.pub_update_lp.publish(e)

    def pubTimingLP(self,timing):
        t = Float64(timing)
        self.pub_timing.publish(t)

    def setError(self,error):
        self.error.data = error

    def pubEndComputeLP(self,status):
        end = Bool()
        end.data = status
        self.pub_end.publish(end)


    #retrieve a single EE goal
    def getDatasEEGoal(self,name_file):
        ee_x = []
        ee_y = []

        name = name_file
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['ee_pos']):
            x = msg.position.x
            ee_x.append(x)
            y = msg.position.y
            ee_y.append(y)
        bag.close() 


        datas = self.scaleDatas(ee_x,ee_y)    

        return datas

    #retrieve a first object position with the angle goal from a single file
    def getInputsInverseModel(self,name_file,goal):
        pos_x = []
        pos_y = []
        angle_goals = []

        name = name_file
        path = os.path.dirname(name)
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['object_pos']):
            x = msg.position.x
            pos_x.append(x)
            y = msg.position.y
            pos_y.append(y)
            angle_goals.append(goal)
        bag.close() 

        datas = self.scaleDatasWithGoal(pos_x,pos_y,angle_goals)    

        return datas

    def getDatasFPosObject(self,name_file):
        pos_x = []
        pos_y = []

        name = name_file
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['object_pos']):
            x = msg.position.x
            pos_x.append(x)
            y = msg.position.y
            pos_y.append(y)
        bag.close() 

        datas = self.scaleDatas(pos_x,pos_y)    

        return datas


    def getDatasLastPosObject(self,name_file):
        pos_x = []
        pos_y = []

        name = name_file
        bag = rosbag.Bag(name)
        for topic, msg, t in bag.read_messages(topics=['object_pos']):
            x = msg.position.x
            pos_x.append(x)
            y = msg.position.y
            pos_y.append(y)
        bag.close() 

        datas = self.scaleDatas(pos_x,pos_y)    

        return datas

    def scaleDatasWithGoal(self,ee_x,ee_y,goals):
        n_x = np.array(ee_x)
        n_x = n_x.reshape(-1,1)
        n_y = np.array(ee_y)
        n_y = n_y.reshape(-1,1)
        n_a = np.array(goals)
        n_a = n_a.reshape(-1,1)

        scaler_x = MinMaxScaler()
        scaler_y = MinMaxScaler()
        scaler_a = MinMaxScaler()
        x_minmax = np.array([0.2, 0.65])
        y_minmax = np.array([-0.5, 0.5])
        a_minmax = np.array([0, 100])
        scaler_x.fit(x_minmax[:, np.newaxis])
        scaler_y.fit(y_minmax[:, np.newaxis])
        scaler_a.fit(a_minmax[:, np.newaxis])
        n_x = scaler_x.transform(n_x)
        n_x = n_x.reshape(1,-1)
        n_x = n_x.flatten()
        n_y = scaler_y.transform(n_y)
        n_y = n_y.reshape(1,-1)
        n_y = n_y.flatten()
        n_a = scaler_a.transform(n_a)
        n_a = n_a.reshape(1,-1)
        n_a = n_a.flatten()
        data = []
        for u,v,w in zip(n_x,n_y,n_a):
            tup = [u,v,w]
            data.append(copy.deepcopy(tup))
        
        return data

    def scaleDatas(self,ee_x,ee_y):
        n_x = np.array(ee_x)
        n_x = n_x.reshape(-1,1)
        n_y = np.array(ee_y)
        n_y = n_y.reshape(-1,1)

        scaler_x = MinMaxScaler()
        scaler_y = MinMaxScaler()
        x_minmax = np.array([0.2, 0.65])
        y_minmax = np.array([-0.5, 0.5])

        scaler_x.fit(x_minmax[:, np.newaxis])
        scaler_y.fit(y_minmax[:, np.newaxis])

        n_x = scaler_x.transform(n_x)
        n_x = n_x.reshape(1,-1)
        n_x = n_x.flatten()
        n_y = scaler_y.transform(n_y)
        n_y = n_y.reshape(1,-1)
        n_y = n_y.flatten()

        data = []
        for u,v in zip(n_x,n_y):
            tup = [u,v]
            data.append(copy.deepcopy(tup))
        
        return data


    #check if we get all 3 samples from directory to avoid mixing old samples with new ones
    def samplesIsComplete(self, list_dir):
        test = ""
        first = True
        complete = True
        for i in list_dir:
            file = os.path.basename(i)
            t = file[:2]
            if first:
                test = t
                first = False
            else:
                if t != test:
                    complete = False
                test = t

        return complete



    def addToMemory(self,sample):
        self.memory.append(sample)
        s = len(self.memory)
        if s > self.memory_size:
            self.memory.pop(0)

    def getMemory(self):
        return self.memory

    def isInMemory(self,sample):
        in_list = False
        if len(self.memory) > 0:
            last_sample = self.memory[-1]
            if torch.equal(last_sample[0][0],sample[0][0]) and torch.equal(last_sample[1][0],sample[1][0]) and torch.equal(last_sample[2][0],sample[2][0]):
                in_list = True

        return in_list

    #Takes angle goal and object location as input and produce a motor command as output
    def trainInverseModel(self):
        current_cost = 0
        last_cost = 15
        learning_rate = 1e-3
        epochs = 300

        #self.inverse_model.to(device)
        criterion = torch.nn.MSELoss()
        optimizer = torch.optim.Adam(self.inverse_model.parameters(),lr=learning_rate)
        #get inputs and targets
        """last_sample = self.memory[-1]
        inputs = last_sample[3][0]
        targets = last_sample[0][0]
        inputs = inputs.to(device)
        targets = targets.to(device)
        for h in range(0,20):
            self.inverse_model.train()
            optimizer.zero_grad()
            outputs = self.inverse_model(inputs)
            cost = criterion(outputs,targets)
            cost.backward()
            optimizer.step()
            current_cost = current_cost + cost.item()"""
            #print("Epoch last sample: {}/{}...".format(h, epochs),
             #                   "MSE : ",current_cost)
            #if current_cost > last_cost:
            #    break
            #last_cost = current_cost
        
        current_cost = 0
        for i in range(0,epochs):
            for j in range(0,len(self.memory)):
                self.inverse_model.train()
                optimizer.zero_grad()
                sample = self.memory[j]
                inputs = sample[3][0]
                targets = sample[0][0]
                inputs = inputs.to(device)
                targets = targets.to(device)
                outputs = self.inverse_model(inputs)
                cost = criterion(outputs,targets)
                cost.backward()
                optimizer.step()
                current_cost = current_cost + cost.item()
            #print("Epoch: {}/{}...".format(i, epochs),
                                #"MSE : ",current_cost)

            if current_cost > last_cost:
                break
            last_cost = current_cost
            current_cost = 0
            
        name = "inverse_epochs_"+str(epochs)+"_lr_"+str(learning_rate)+".pt"

        #torch.save(self.inverse_model, name)

    #concatenate first object position with motor command (goal EE)
    def getInputsForwardModel(self,goal_ee,fpos):
        return torch.cat((goal_ee,fpos),0)

    #takes object location and motor command as input and produces the expected future object location as output
    def trainForwardModel(self):
        current_cost = 0
        last_cost = 15
        learning_rate = 1e-3
        epochs = 300
        data_input = []

        self.forward_model.to(device)
        criterion = torch.nn.MSELoss()
        optimizer = torch.optim.Adam(self.forward_model.parameters(),lr=learning_rate)
        #get inputs and targets
        """last_sample = self.memory[-1]
        inputs = self.getInputsForwardModel(last_sample[0][0],last_sample[1][0])
        targets = last_sample[2][0]
        inputs = inputs.to(device)
        targets = targets.to(device)
        for h in range(0,20):
            self.forward_model.train()
            optimizer.zero_grad()
            outputs = self.forward_model(inputs)
            cost = criterion(outputs,targets)
            cost.backward()
            optimizer.step()
            current_cost = current_cost + cost.item()"""
            #print("Epoch last sample: {}/{}...".format(h, epochs),
                               # "MSE : ",current_cost)
            #last_cost = current_cost
        current_cost = 0
        for i in range(0,epochs):
            for j in range(0,len(self.memory)):
                self.forward_model.train()
                optimizer.zero_grad()
                sample = self.memory[j]
                inputs = self.getInputsForwardModel(sample[0][0],sample[1][0])
                targets = sample[2][0]
                inputs = inputs.to(device)
                targets = targets.to(device)
                outputs = self.forward_model(inputs)
                cost = criterion(outputs,targets)
                cost.backward()
                optimizer.step()
                current_cost = current_cost + cost.item()
            #print("Epoch: {}/{}...".format(i, epochs),
                                #"MSE : ",current_cost)

            if current_cost > last_cost:
                break
            last_cost = current_cost
            current_cost = 0
            
        name = "forward_epochs_"+str(epochs)+"_lr_"+str(learning_rate)+".pt"

        #torch.save(self.forward_model, name)
    
    #compute the error between the prediction and the actual data
    def getErrorPrediction(self,prediction,actual):
        error = math.sqrt((actual[0]*1-prediction[0]*1)**2 + (actual[1]*1-prediction[1]*1)**2)

        return error

    #return the error between prediction and actual motor command for the inverse model
    #parameters are tensors
    def predictInverseModel(self,inputs,targets):
        self.inverse_model.eval()
        inputs = inputs.to(device)
        targets = targets.to(device)
        out = self.inverse_model(inputs)
        error = self.getErrorPrediction(out,targets)

        return error

    def predictForwardModel(self,inputs,targets):
        self.forward_model.eval()
        inputs = inputs.to(device)
        targets = targets.to(device)
        out = self.forward_model(inputs)
        error = self.getErrorPrediction(out,targets)

        return error

    def getTensorsFromSamples(self,list_samples,goal):
        samples = []
        for i in list_samples:
            if "goalmotion" in i:
                data_goals = self.getDatasEEGoal(i)
                tensor_goals = torch.tensor(data_goals,dtype=torch.float)
            if "objectfpos" in i:
                data_inv = self.getInputsInverseModel(i,goal)
                tensor_inputs_inverse = torch.tensor(data_inv,dtype=torch.float)
                data_fpos = self.getDatasFPosObject(i)
                tensor_fpos = torch.tensor(data_fpos,dtype=torch.float)
            if "objectlastpos" in i :
                data_lpos = self.getDatasLastPosObject(i)
                tensor_lpos = torch.tensor(data_lpos,dtype=torch.float)
        
        samples.append(tensor_goals)
        samples.append(tensor_fpos)
        samples.append(tensor_lpos)
        samples.append(tensor_inputs_inverse)

        return samples


def samplesComplete(list_dir):
    test = ""
    first = True
    complete = True
    for i in list_dir:
        file = os.path.basename(i)
        t = file[:2]
        if first:
            test = t
            first = False
        else:
            if t != test:
                complete = False
            test = t

    return complete


def getLatestComingSamples():
    samples = []
    last_samples = []
    normalized_path = ""
    complete_dir = False
    complete_samples = False
    lpos_in = False
    fpos_in = False
    goal_in = False
    all_in = False
    empty = False
    while not complete_samples:
        obg, name = getLatestModifiedDirectory()
        if name:
            name = name + "/*"
            files = glob.glob(os.path.expanduser(name))
            dir_samples = sorted(files, key=lambda t: os.stat(t).st_mtime)
            if len(dir_samples) % 3 == 0 and len(dir_samples) > 0:
                last_samples.append(dir_samples.pop())
                last_samples.append(dir_samples.pop())
                last_samples.append(dir_samples.pop())
                complete_samples = samplesComplete(last_samples)

    return last_samples, obg, complete_samples

def sampleInList(l,s):
    check = False
    if s in l:
        check = True
    
    return check


def getLatestModifiedDirectory():
    t = 0
    last_file = ""
    n_dir = ""
    obg = ObjectGoal()
    files = glob.glob(os.path.expanduser("/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/datas/*"))
    sorted_by_mtime_ascending = sorted(files, key=lambda t: os.stat(t).st_mtime)
    for i in range(0,len(sorted_by_mtime_ascending)):
        object_directory = sorted_by_mtime_ascending[i] + "/*"
        files_goals = glob.glob(os.path.expanduser(object_directory))
        dir_goals = sorted(files_goals, key=lambda t: os.stat(t).st_mtime)
        for j in range(0,len(dir_goals)):
            list_files = dir_goals[j] + "/*"
            files = glob.glob(os.path.expanduser(list_files))
            dir_files = sorted(files, key=lambda t: os.stat(t).st_mtime)
            for k in range(0,len(dir_files)):
                p = Path(dir_files[k])
                last_mod = p.stat().st_mtime
                if last_mod > t:
                    last_file = dir_files[k]
                    t = last_mod
    if last_file != "":
        normalized_path = os.path.normpath(last_file)
        n_dir = os.path.dirname(normalized_path)
        path_components = normalized_path.split(os.sep)
        tmp_goal = path_components[-2]
        tmp_obj = path_components[-3]
        tmp_goal = float(tmp_goal.split("-",1)[1])
        tmp_obj = float(tmp_obj.split("-",1)[1])
        obg.object = tmp_obj
        obg.goal = tmp_goal

    return obg, n_dir

def modelObjectExists(o,l):
    o = int(o)
    exists = False
    for i in range(0,len(l)):
        #print("----------")
        #print(o)
        #print(l[i].getModObject())
        #print("-------")
        if o == l[i].getModObject():
            exists = True
        else:
            exists = False

    return exists

def indexOfmodel(o,l):
    o = int(o)
    ind = -1
    for i in range(0,len(l)):
        if o == l[i].getModObject():
            ind = i

    return ind


if __name__ == "__main__":
    last_samples = []
    object_models = []
    old_sample = ""
    object_goal = ObjectGoal()
    old_og = ObjectGoal()
    first = True
    complete = False
    torch.manual_seed(42)
    #cylinder = Skill(3,6,2,4,8,2,20)

    while not rospy.is_shutdown():
        
        last_samples, object_goal, complete = getLatestComingSamples()
        #print(last_samples)
        inList = sampleInList(last_samples,old_sample)
        if complete and (not inList or (old_og.goal != object_goal.goal or old_og.object != object_goal.object)):
            print(object_goal)
            old_sample = last_samples[0]
            old_og.goal = object_goal.goal
            old_og.object = object_goal.object
            e = modelObjectExists(object_goal.object,object_models)
            if not e:
                print("Model does not exist, create one...")
                mo = Skill(3,6,2,4,8,2,40,object_goal.object)
                object_models.append(mo)
            ind = indexOfmodel(object_goal.object,object_models)
            rospy.sleep(0.5)
            last_tensor = object_models[ind].getTensorsFromSamples(last_samples,object_goal.goal)
            #print(tensor_model)
            print("Tensor goal motion : ",last_tensor[0][0])
            print("Tensor input first pos object : ",last_tensor[1][0])
            print("Tensor last pos : ",last_tensor[2][0])
            print("Tensor input inverse model : ",last_tensor[3][0])
            if object_models[ind].isInMemory(last_tensor) == False:
                object_models[ind].addToMemory(last_tensor)
                #error_inv = object_models[ind].predictInverseModel(last_tensor[3][0],last_tensor[0][0])
                inp_fwd = object_models[ind].getInputsForwardModel(last_tensor[0][0],last_tensor[1][0])
                error_fwd = object_models[ind].predictForwardModel(inp_fwd,last_tensor[2][0])
                #error_fwd = math.tanh(error_fwd)
                #print("ERROR INVERSE : ",error_inv)
                #print("ERROR FORWARD : ",error_fwd)         
                #total_error = error_inv + error_fwd
                print("ERROR Forward : ",error_fwd)
                print("MEMORY MODEL : ",len(object_models[ind].getMemory()))
                object_models[ind].pubTimingLP(1.0)
                object_models[ind].pubUpdateLP(error_fwd)
                rospy.sleep(1)
                object_models[ind].pubTimingLP(0.0)
                object_models[ind].pubUpdateLP(0)
                object_models[ind].pubEndComputeLP(True)
                rospy.sleep(1)
                object_models[ind].pubEndComputeLP(False)
                object_models[ind].trainInverseModel()
                object_models[ind].trainForwardModel()
                print("number of models : ",len(object_models))





