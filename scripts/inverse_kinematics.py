#rafi python try 3

#negative phi 2

import rospy
from std_msgs.msg import Int16MultiArray

import math
from colorama import Fore, Style


class RosConnection:
    def __init__(self) -> None:

        self.pub = rospy.Publisher('cmd_angle', Int16MultiArray, queue_size=10)
        rospy.init_node('angle_controller_ik', anonymous=True)
        self.rate = rospy.Rate(10)
        self.busy = False
        self.coords_to_send = []
        self.origin = (700,10)
        self.HEIGHT = 15 # 15 cm
        self.NAMES = {
            'paper': 1,
            'paper_cup' : 1,
            'plastic_bottle': 2,
            'chips_packet': 2,
            'can': 3
        }

    def send_coords(self, coords):
        # @param coords= ['name', x, y]
        self.coords_to_send += coords
        self.busy = True
        print('before for loop')
        for i, (nm, x, y) in enumerate(self.coords_to_send):
            msg = Int16MultiArray()
            print('before try')
            try:
                msg.data = [self.NAMES[nm], *self.calculate_base_rotation(nm, x, y)]
                self.pub(msg)
                print(f"in_send_coords: published")
            except TypeError as e:
                print(f"{Fore.LIGHTRED_EX}In SEND_COORDS:{e}{Style.RESET_ALL}")
                pass
        self.busy = False
            

    def calculate_base_rotation(self, name, x, y):
        # base_angle = msg.data[0];
        # shoulder_angle = msg.data[1];
        # elbow_angle = msg.data[2];
        # pitch_angle = msg.data[3];
        # grip_close = msg.data[4] > 0;
        base_angle = math.atan((self.origin[0]-x) / (y-self.origin[1]))
        r = math.sqrt((self.origin[0]-x)**2 + (self.origin[1]-y)**2)
        r = r - 16 # 16 cm back from actual object
        z = 5 - self.HEIGHT #Depth from base
        try:
            shoulder_angle, elbow_angle = self.inv_kinematics(r, z)
            pitch_angle = 30
            return base_angle, shoulder_angle, elbow_angle, pitch_angle
        except TypeError as e:
            print(f"{Fore.LIGHTRED_EX}In CALC_BASE_ROTATION:{e}{Style.RESET_ALL}")
            return None
        


            
    def inv_kinematics(self, r, z, l1=10, l2=10):
        if not (-1 <= (r**2+z**2-l1**2-l2**2)/(2*l1*l2) <= 1):
            # domain test of acos
            # also limit that is possible for the two links to reach
            return

        # Equations for Inverse kinematics
        phi_2 = (math.acos((r**2+z**2-l1**2-l2**2)/(2*l1*l2)))  # eqn 2
        phi_1 = math.atan2(z, r) + math.atan2(l2*math.sin(phi_2), (l1 + l2*math.cos(phi_2))) # eqn 3
        phi_2 = -phi_2
        #theta_1 = rad2deg(phi_2-phi_1)  # eqn 4 converted to degrees

        #phi_3 = arccos((r1**2-arm1**2-arm1**2)/(-2*arm1*arm2))
        #theta_2 = 180-rad2deg(phi_3)

        theta1 = math.degrees(phi_1)
        theta2 = math.degrees(phi_2)

        #print('theta 1: ', phi_1)
        #print('theta two: ', phi_2)

        return theta1, theta2


