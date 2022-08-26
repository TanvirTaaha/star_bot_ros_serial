#in this code we are calculating values for a robot which has just two joints

import numpy as np

#defining the lengths of the links in cm

ay1 = 11
az1 = 0.3
ax1 = -2.05
ay2 = 18.75
az2 = 2

#taking angles in degrees
T1 = 0
T2 = 75

#converting angles to radians
T1 = (T1/180)*np.pi
T2 = (T2/180)*np.pi

#rotating matrices
R0_1a = [[np.cos(T1), -np.sin(T1), 0], [np.sin(T1), np.cos(T1), 0], [0, 0, 1]]
R0_1b = [[1,0,0],[0,1,0],[0,0,1]]
R0_1 = np.dot(R0_1a, R0_1b)
R1_2a = [[np.cos(T2), -np.sin(T2), 0], [np.sin(T2), np.cos(T2), 0], [0, 0, 1]]
R1_2b = [[1,0,0], [0,1,0], [0,0,1]]
R1_2 = np.dot(R1_2b, R1_2a)

#multiplying joint 1 and joint 2 to get the rotation of link 2 w.r.t base joint.
R0_2 = np.dot(R0_1, R1_2)

#print (np.matrix(R0_1))

#Now we will calculate the displacement vectors

d0_1 = [[ax1], [ay1], [az1]]
d1_2 = [[ay2*np.cos(T2)], [ay2*np.sin(T2)], [az2]]

#print(np.matrix(d0_1))
#print(np.matrix(d1_2))

#now we will create our Homogenous Transformation Matrices

H0_1 = np.concatenate((R0_1, d0_1), 1)
H0_1 = np.concatenate((H0_1, [[0,0,0,1]]), 0)

H1_2 = np.concatenate((R1_2, d1_2), 1)
H1_2 = np.concatenate((H1_2, [[0,0,0,1]]), 0)

H0_2 = np.dot(H0_1, H1_2)

print (np.matrix(H0_2))