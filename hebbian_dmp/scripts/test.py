#! /usr/bin/python3
import numpy as np

current_field = np.zeros((4,4))
dmp_neurons = np.zeros((1,4))
weights = np.zeros((2,4,4))

current_field = np.array([[0, 0, 0 ,0], [0, 0, 0, 0],[0,0,0,0],[0,0,0,1]])
other_field = np.array([[0, 0, 0 ,0], [0, 1, 0, 0],[0,0,0,0],[0,0,0,0]])
dmp_neurons = np.array([1,1,1,1])
print(dmp_neurons)
t = np.multiply(dmp_neurons,current_field)

f = np.multiply(other_field,current_field)

#print(t)

#print(f)

print(weights.shape[2])
