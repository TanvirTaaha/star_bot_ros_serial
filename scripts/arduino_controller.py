import os
import rospy
from std_msgs.msg import Int16MultiArray, Int16
from colorama import Fore, Style


root_dir = os.path.join('/Users', 'taaha', 'codes', 'ros', 'project_ws/src', 'ros_serial_test', 'scripts')  
class ArduinoController:
    def arduino_callback(self, msg: Int16):
        if(msg.data == 1): # Arduino reached
            rospy.loginfo(f"got 1 from arduino")
            self.seq_pos += 1
            if len(self.get_angles()) == self.seq_pos: # all sequence done
                self.dest = -1
                self.seq_pos = 0
                self.angle_msg.data = []
                return
            # seq still left
            msg = Int16MultiArray()
            msg.data = self.get_angles()[self.seq_pos]
            rospy.loginfo(f"get_angles:{msg.data}")

            self.pub.publish(msg)
            rospy.loginfo(f"msg:{msg}")


        

    def __init__(self):
        self.seq_pos = 0
        self.angle_msg = Int16MultiArray()
        self.dest = -1
    
        self.pub = rospy.Publisher('arduino_input', Int16MultiArray, queue_size=10)
        self.sub = rospy.Subscriber('arduino_output', Int16, callback=self.arduino_callback)
        self.ik_sub = rospy.Subscriber('cmd_angle', Int16MultiArray, callback=self.angle_callback)
        rospy.init_node('arduino_python_controller', anonymous=True, log_level=rospy.DEBUG)


    def angle_callback(self, msg: Int16MultiArray):
        if self.dest == -1 and len(msg.data) == 5: # [nm, base_angle, shoulder_angle, elbow_angle, pitch_angle]
            self.dest = msg.data[0]
            self.angle_msg.data = [*msg.data[1:], 1] # angle_msg.data += [1]
            rospy.loginfo(f'{Fore.CYAN}dest was -1, got msg:{msg.data}{Style.RESET_ALL}')
            rospy.loginfo(f'{Fore.CYAN}dest was -1, set angle_msg:{self.angle_msg.data}{Style.RESET_ALL}')
            if self.seq_pos != 0: 
                rospy.logfatal(f"dest == -1, but seq_pos:{self.seq_pos}")
            else:
                m = Int16()
                m.data = 1
                self.arduino_callback(m) # calling first time 

    def get_angles(self):
        pick_up = [110, 100, 30]
        put_down = [30, 80, 30]
        B_ANGLES = {
            -1: 0, # default
            1: 75,
            2: -75,
            3: 180
        }
        seq = [
            [*self.angle_msg.data], # go as per ik and grip(as set in angle_callback)
            [self.angle_msg.data[0], *pick_up, 1], # at the same base(obj) pick obj up, grip close
            [B_ANGLES[self.dest], *pick_up, 1], # rotate while at pickup state 
            [B_ANGLES[self.dest], *put_down, 0], # put down at dest location
            [B_ANGLES[self.dest], *pick_up, 0], # pick up arm 
            [B_ANGLES[-1], *pick_up, 0] # go to default position
        ]
        return seq

    def run(self):
        while not rospy.is_shutdown():
            if self.dest == -1:
                if self.seq_pos != 0 and len(self.angle_msg.data) != 0:
                    rospy.logfatal(f"dest == -1 but \n\tseq_pos:{self.seq_pos},\n\tangle_msg:{self.angle_msg.data}")
                else:
                    rospy.loginfo(f"No command state..listening..")
            else:
                angles = self.get_angles()[self.seq_pos]
                rospy.loginfo(f"dest:{self.dest}, seq:{self.seq_pos}, angles:{angles}")
            
            rate = rospy.Rate(2)
            # if self.dest != -1:
            #     msg = Int16MultiArray()
            #     msg.data = self.get_angles()[self.seq_pos]
            #     rospy.loginfo(f"get_angles:{msg.data}")

            #     self.pub.publish(msg)
            #     rospy.loginfo(f"msg:{msg}")

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = ArduinoController()
        controller.run()
    except rospy.ROSInterruptException as e:
        print(e)