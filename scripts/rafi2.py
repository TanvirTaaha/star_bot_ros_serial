#rafi python try 2

#positive phi 2
from numpy import *

a1 = 10  # length of link arm1 in cm
a2 = 10  # length of link arm2 in cm

# Desired Position of End effector
x = 10
y = 17

r = sqrt(x*x + y*y)
print(f"radius: {r}")

# Equations for Inverse kinematics
phi_2 = arccos((x**2+y**2-a1**2-a2**2)/(2*a1*a2))  # eqn 2
phi_1 = arctan2(y, x) - arctan2(a2*sin(phi_2), (a1 + a2*cos(phi_2))) # eqn 3
#theta_1 = rad2deg(phi_2-phi_1)  # eqn 4 converted to degrees

#phi_3 = arccos((r1**2-arm1**2-arm1**2)/(-2*arm1*arm2))
#theta_2 = 180-rad2deg(phi_3)

theta1 = rad2deg(phi_1)
theta2 = rad2deg(phi_2)

#print('theta 1: ', phi_1)
#print('theta two: ', phi_2)


print('theta 1: ', theta1)
print('theta two: ', theta2)