#!/usr/bin/env python3

import copy

#from torch._C import T
import rospy
from std_msgs.msg import Float64
import rospy
import numpy as np
import math
from std_msgs.msg import Bool
import torch
import torch.nn as nn
from sklearn.preprocessing import MinMaxScaler
from torch.autograd import Variable
from scipy.ndimage import gaussian_filter1d
from perception.msg import ObjectGoal
from perception.msg import ObjectController
from geometry_msgs.msg import Pose
import geometry_msgs.msg
try:
    import cPickle as pickle
except ModuleNotFoundError:
    import pickle
from os import path
import os
#from visdom import Visdom


# torch.cuda.is_available() checks and returns a Boolean True if a GPU is available, else it'll return False
is_cuda = torch.cuda.is_available()

# If we have a GPU available, we'll set our device to GPU. We'll use this device variable later in our code.
if is_cuda:
    device = torch.device("cpu")
    print("GPU is available but use CPU")
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
    def __init__(self,input_layer_iv,middle_layer_iv,output_layer_iv,input_layer_fw,middle_layer_fw,output_layer_fw,mem_size,object_goal):
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
        self.model_object = ObjectGoal()
        self.model_object.object = object_goal.object
        self.model_object.goal = object_goal.goal
        self.memory = []

    def getModObject(self):
        return self.model_object

    def addToMemory(self,sample):
        self.memory.append(sample)
        s = len(self.memory)
        if s > self.memory_size:
            self.memory.pop(0)

    def getMemory(self):
        return self.memory


    #Takes angle goal and object location as input and produce a motor command as output
    def trainInverseModel(self):
        current_cost = 0
        last_cost = 15
        learning_rate = 5e-3
        epochs = 5

        #self.inverse_model.to(device)
        criterion = torch.nn.MSELoss()
        optimizer = torch.optim.Adam(self.inverse_model.parameters(),lr=learning_rate)        
        current_cost = 0
        for i in range(0,10):
            self.inverse_model.train()
            optimizer.zero_grad()
            sample = self.memory[-1]
            inputs = sample[0]
            targets = sample[1]
            inputs = inputs.to(device)
            targets = targets.to(device)
            outputs = self.inverse_model(inputs)
            cost = criterion(outputs,targets)
            cost.backward()
            optimizer.step()
            #current_cost = current_cost + cost.item()
        for i in range(0,epochs):
            for j in range(0,len(self.memory)):
                self.inverse_model.train()
                optimizer.zero_grad()
                sample = self.memory[j]
                inputs = sample[0]
                targets = sample[1]
                inputs = inputs.to(device)
                targets = targets.to(device)
                outputs = self.inverse_model(inputs)
                cost = criterion(outputs,targets)
                cost.backward()
                optimizer.step()
                #current_cost = current_cost + cost.item()
            #print("Epoch: {}/{}...".format(i, epochs),
                                #"MSE : ",current_cost)

            #if current_cost > last_cost:
            #    break
            #last_cost = current_cost
            #current_cost = 0
            
        #name = "inverse_epochs_"+str(epochs)+"_lr_"+str(learning_rate)+".pt"

        #torch.save(self.inverse_model, name)

    #takes object location and motor command as input and produces the expected future object location as output
    def trainForwardModel(self):
        current_cost = 0
        last_cost = 15
        learning_rate = 5e-5
        epochs = 5
        data_input = []
        self.forward_model.to(device)
        criterion = torch.nn.MSELoss()
        optimizer = torch.optim.Adam(self.forward_model.parameters(),lr=learning_rate)
        current_cost = 0
        for i in range(0,10):
            self.forward_model.train()
            optimizer.zero_grad()
            sample = self.memory[-1]
            inputs = sample[2]
            targets = sample[3]
            inputs = inputs.to(device)
            targets = targets.to(device)
            outputs = self.forward_model(inputs)
            cost = criterion(outputs,targets)
            cost.backward()
            optimizer.step()
            current_cost = current_cost + cost.item()
        for i in range(0,epochs):
            for j in range(0,len(self.memory)):
                self.forward_model.train()
                optimizer.zero_grad()
                sample = self.memory[j]
                inputs = sample[2]
                targets = sample[3]
                inputs = inputs.to(device)
                targets = targets.to(device)
                outputs = self.forward_model(inputs)
                cost = criterion(outputs,targets)
                cost.backward()
                optimizer.step()
                #current_cost = current_cost + cost.item()
            #print("Epoch: {}/{}...".format(i, epochs),
                                #"MSE : ",current_cost)

            #if current_cost > last_cost:
            #    break
            #last_cost = current_cost
            #current_cost = 0
            
        #name = "forward_epochs_"+str(epochs)+"_lr_"+str(learning_rate)+".pt"

        #torch.save(self.forward_model, name)
    
    #compute the error between the prediction and the actual data
    def getErrorPrediction(self,prediction,actual):
        error = math.sqrt((actual[0][0]-prediction[0][0])**2 + (actual[0][1]-prediction[0][1])**2)

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

    def predictIVModel(self,inputs):
        self.inverse_model.eval()
        inputs = inputs.to(device)
        out = self.inverse_model(inputs)

        return out

    def predictForwardModel(self,inputs,targets):
        self.forward_model.eval()
        inputs = inputs.to(device)
        targets = targets.to(device)
        out = self.forward_model(inputs)
        error = self.getErrorPrediction(out,targets)

        return error

    def saveMemory(self):
        #name = "/home/altair/PhD/catkin_noetic/rosbags/experiment/datas/goal_"+str(int(self.model_object.object))+"_"+str(int(self.model_object.goal))+".pkl"
        name = "/home/altair/PhD/Codes/ExperimentIM-LCNE/goal_"+str(int(self.model_object.object))+"_"+str(int(self.model_object.goal))+".pkl"
        exist = path.exists(name)
        if exist:
            os.remove(name)
        filehandler = open(name, 'wb')
        pickle.dump(self.memory, filehandler)
        #with open(name, 'wb') as outp:
        #    pickle.dump(self.memory, outp, pickle.HIGHEST_PROTOCOL)

    def retrieveMemory(self):
        #name = "/home/altair/PhD/catkin_noetic/rosbags/experiment/datas/goal_"+str(int(self.model_object.object))+"_"+str(int(self.model_object.goal))+".pkl"
        name = "/home/altair/PhD/Codes/ExperimentIM-LCNE/goal_"+str(int(self.model_object.object))+"_"+str(int(self.model_object.goal))+".pkl"
        #with open(name, 'rb') as inp:
        #    mem = pickle.load(inp)
        filehandler = open(name, 'rb') 
        mem = pickle.load(filehandler)
        self.memory = mem


class RosConnection(object):
    def __init__(self):
        self.pose_b = geometry_msgs.msg.Pose()
        rospy.init_node('skills', anonymous=True)
        rospy.Subscriber("/intrinsic/controller", ObjectController, self.callbackController)
        rospy.Subscriber("/intrinsic/learning_signal", Float64, self.callbackLearning)
        rospy.Subscriber("/intrinsic/og_pred", ObjectGoal, self.callbackObjectGoal)
        rospy.Subscriber("/cylinder_position", Pose, self.callbackObjectPosition)
        self.pub_update_lp = rospy.Publisher('/intrinsic/learning_progress', Float64, latch=True, queue_size=10)
        self.pub_timing = rospy.Publisher('/intrinsic/updating_lp', Float64, latch=True, queue_size=10)
        self.pub_end = rospy.Publisher('/intrinsic/end_action', Bool, queue_size=10)
        self.pub_new = rospy.Publisher('/intrinsic/new_goal', Pose, queue_size=10)
        self.data_controller = ObjectController()
        self.iv_og = ObjectGoal()
        self.iv_og.object = -1.0
        self.data_controller.object_model.object = -1.0
        self.learning = False
        self.info = False

    def callbackController(self,data):
        self.data_controller.object_model.object = data.object_model.object
        self.data_controller.object_model.goal = data.object_model.goal
        self.data_controller.ee_last_pos.position.x = data.ee_last_pos.position.x
        self.data_controller.ee_last_pos.position.y = data.ee_last_pos.position.y
        self.data_controller.object_fpos.position.x = data.object_fpos.position.x
        self.data_controller.object_fpos.position.y = data.object_fpos.position.y
        self.data_controller.object_lpos.position.x = data.object_lpos.position.x
        self.data_controller.object_lpos.position.y = data.object_lpos.position.y

    def callbackObjectPosition(self,p):
        self.pose_b.position.x = p.position.x
        self.pose_b.position.y = p.position.y
        self.pose_b.position.z = p.position.z

    def callbackLearning(self,l):
        if l.data > 0:
            self.learning = True
        else:
            self.learning = False
        self.info = True

    def callbackObjectGoal(self,og):
        self.iv_og.object = og.object
        self.iv_og.goal = og.goal

    def getObjectGoal(self):
        return self.iv_og

    def getBlueObjectPosition(self):
        return self.pose_b

    def pubUpdateLP(self,error):
        e = Float64(error)
        self.pub_update_lp.publish(e)

    def pubTimingLP(self,timing):
        t = Float64(timing)
        self.pub_timing.publish(t)

    def pubEndComputeLP(self,status):
        end = Bool()
        end.data = status
        self.pub_end.publish(end)

    def pubNewGoal(self,og):
        p = geometry_msgs.msg.Pose()
        p.position.x = og.object
        p.position.y = og.goal
        self.pub_new.publish(p)

    def getDataController(self):
        return self.data_controller

    def getLearning(self):
        return self.learning, self.info

    def resetDatas(self):
        self.data_controller = ObjectController()
        self.data_controller.object_model.object = -1.0

    def resetObjectGoal(self):
        self.iv_og = ObjectGoal()
        self.iv_og.object = -1.0

    def resetLearning(self):
        self.learning = False
        self.info = False

#concatenate first object position with motor command (goal EE)
def getInputsForwardModel(goal_ee,fpos):
    return torch.cat((goal_ee,fpos),1)

def scaleDatas(ee_x,ee_y):
    n_x = np.array(ee_x)
    n_x = n_x.reshape(-1,1)
    n_y = np.array(ee_y)
    n_y = n_y.reshape(-1,1)
    scaler_x = MinMaxScaler()
    scaler_y = MinMaxScaler()
    x_minmax = np.array([0.18, 0.67])
    y_minmax = np.array([-0.35, 0.35])
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

def getTensors(datas):
    sample = []
    ee_x = []
    ee_y = []
    fpos_x = []
    fpos_y = []
    lpos_x = []
    lpos_y = []
    ee_x.append(datas.ee_last_pos.position.x)
    ee_y.append(datas.ee_last_pos.position.y)
    fpos_x.append(datas.object_fpos.position.x)
    fpos_y.append(datas.object_fpos.position.y)
    lpos_x.append(datas.object_lpos.position.x)
    lpos_y.append(datas.object_lpos.position.y)
    scaled_ee = scaleDatas(ee_x,ee_y)
    scaled_fpos = scaleDatas(fpos_x,fpos_y)
    scaled_lpos = scaleDatas(lpos_x,lpos_y)
    tensor_ee_lpos = torch.tensor(scaled_ee,dtype=torch.float)
    tensor_object_fpos = torch.tensor(scaled_fpos,dtype=torch.float)
    tensor_object_lpos = torch.tensor(scaled_lpos,dtype=torch.float)
    tensor_input_fwd = getInputsForwardModel(tensor_ee_lpos,tensor_object_fpos)
    sample.append(tensor_object_fpos)
    sample.append(tensor_ee_lpos)
    sample.append(tensor_input_fwd)
    sample.append(tensor_object_lpos)

    return sample

def UnScaleDatasFromTensor(tensor_):
    tmp = tensor_[0].tolist()
    n_x = np.array(tmp[0])
    n_x = n_x.reshape(-1,1)
    n_y = np.array(tmp[1])
    n_y = n_y.reshape(-1,1)
    scaler_x = MinMaxScaler(feature_range=(0.18,0.67))
    scaler_y = MinMaxScaler(feature_range=(-0.35,0.35))
    x_minmax = np.array([0, 1])
    y_minmax = np.array([0, 1])
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
        
    return data[0]

def getTensorObject(point):
    fpos_x = []
    fpos_y = []
    fpos_x.append(point.position.x)
    fpos_y.append(point.position.y)
    scaled_fpos = scaleDatas(fpos_x,fpos_y)
    tensor_object_fpos = torch.tensor(scaled_fpos,dtype=torch.float)

    return tensor_object_fpos

def goalObjectExist(list_g,oc):
    exists = False
    index = -1
    for i in range(0,len(list_g)):
        if (abs(int(list_g[i].model_object.object) - int(oc.object_model.object)) < 3) and (abs(int(list_g[i].model_object.goal) - int(oc.object_model.goal)) < 3):
            exists = True
            index = i
            oc.object_model.object = list_g[i].model_object.object
            oc.object_model.goal = list_g[i].model_object.goal
        else:
            exists = False

    return exists, index, oc

def getNNObject(list_g,og):
    exists = False
    index = -1
    for i in range(0,len(list_g)):
        if (abs(int(list_g[i].model_object.object) - int(og.object)) < 3) and (abs(int(list_g[i].model_object.goal) - int(og.goal)) < 3):
            exists = True
            index = i
            og.object = list_g[i].model_object.object
            og.goal = list_g[i].model_object.goal
        else:
            exists = False

    return exists, index, og


if __name__ == "__main__":
    nn_models = []
    data_c = ObjectController()
    torch.manual_seed(42)
    rc = RosConnection()
    rate = rospy.Rate(100)
    training = False
    iv_object = ObjectGoal()
    iv_object.object = -1.0
    #cylinder = Skill(3,6,2,4,8,2,20)

    while not rospy.is_shutdown():
        data_c = rc.getDataController()
        iv_object = rc.getObjectGoal()
        if data_c.object_model.object > 0.0:
            e, ind, data_c = goalObjectExist(nn_models,data_c)
            #print(data_c)
            if not e:
                print("Model does not exist, create one...")
                mo = Skill(2,3,2,4,8,2,20,data_c.object_model)
                nn_models.append(mo)
                ind = len(nn_models) - 1
                rc.pubNewGoal(data_c.object_model)
            sample = getTensors(data_c)
            #print("Tensor input first pos object : ",sample[0])
            #print("Tensor last pos : ",sample[3])
            #print("Tensor input inverse model : ",sample[2])
            nn_models[ind].addToMemory(sample)
            error_fwd = nn_models[ind].predictForwardModel(sample[2],sample[3])
            error_inv = nn_models[ind].predictInverseModel(sample[0],sample[1])
            #error_fwd = math.tanh(error_fwd)
            print("ERROR INVERSE : ",error_inv)
            print("ERROR FORWARD : ",error_fwd)         
            print("MEMORY MODEL : ",len(nn_models[ind].getMemory()))
            rc.pubUpdateLP(error_fwd)
            rc.pubTimingLP(1.0)
            rospy.sleep(1)
            rc.pubUpdateLP(0.0)
            rc.pubTimingLP(0.0)
            rc.pubEndComputeLP(True)
            rospy.sleep(1)
            rc.pubEndComputeLP(False)
            nn_models[ind].trainInverseModel()
            nn_models[ind].trainForwardModel()
            """while not training:
                l, inf = rc.getLearning()
                if inf == True:
                    if l == True:
                        print("SKILLS : Training...")
                        
                    else:
                        print("SKILLS : No Training")
                    rc.resetLearning()
                    training = True"""
            print("number of models : ",len(nn_models))
            nn_models[ind].saveMemory()
            rc.resetDatas()
            data_c = ObjectController()
            training = False
        if iv_object.object != -1.0:
            tmp_og = ObjectGoal()
            e, ind, tmp_og = getNNObject(nn_models,iv_object)
            if e:
                p = geometry_msgs.msg.Pose()
                p = rc.getBlueObjectPosition()
                sample = getTensorObject(p)
                prediction = nn_models[ind].predictIVModel(sample)
                coord = UnScaleDatasFromTensor(prediction)
                og = ObjectGoal()
                og.object = coord[0]
                og.goal = coord[1]
        rc.resetObjectGoal()
        rate.sleep()





