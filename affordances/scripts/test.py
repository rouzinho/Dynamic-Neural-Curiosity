#!/usr/bin/env python3

from skills_direction import *

def getFileInList():
    name = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/"
    list_models = []
    onlyfiles = [f for f in listdir(name) if isfile(join(name, f))]
    for i in onlyfiles:
        if ".pt" in i:
            n = name + str(i)
            list_models.append(n)
    return list_models

def extractName(name):
    g = name[-5:-3]
    o = name[-8:-6]
    return o,g

def loadNNDatas():
    l = getFileInList()
    nn = []
    for i in l:
        o, g = extractName(i)
        ob = ObjectGoal()
        ob.object = float(o)
        ob.goal = float(g)
        if "forward" in i:
            mo = Skill(3,4,2,4,6,1,20,ob)
            mo.forward_model = torch.load(i)
            name_inv = "/home/altair/PhD/Codes/ExperimentIM-LCNE/datas/complete/inverse_"+o+"_"+g+".pt"
            mo.inverse_model = torch.load(name_inv)
            mo.retrieveMemory()
            nn.append(mo)
    return nn

if __name__ == '__main__':
    #t = getFileInList()
    t = loadNNDatas()
    for i in t:
        print(i.memory)