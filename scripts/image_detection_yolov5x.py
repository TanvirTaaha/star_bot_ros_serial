import argparse
from statistics import mode
import torch
from matplotlib import pyplot as plt
import cv2
from pathlib import Path
import numpy as np

import rospy
from std_msgs.msg import Int16MultiArray

import math
from colorama import Fore, Style


class RosConnection:
    def __init__(self) -> None:

        self.pub = rospy.Publisher('cmd_angle', Int16MultiArray, queue_size=10)
        rospy.init_node('angle_controller_ik', anonymous=True, log_level=rospy.DEBUG)
        self.rate = rospy.Rate(1)
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
        rospy.loginfo('before for loop')
        for i, (nm, x, y) in enumerate(self.coords_to_send):
            msg = Int16MultiArray()
            rospy.loginfo('before try')
            try:
                msg.data = [self.NAMES[nm], *self.calculate_base_rotation(nm, x, y)]
                self.pub(msg)
                self.rate.sleep()
                self.pub(msg)
                self.rate.sleep()
                print(f"in_send_coords: published")
                rospy.loginfo('in_send_coords: published')
            except TypeError as e:
                print(f"{Fore.LIGHTRED_EX}In SEND_COORDS:{e}{Style.RESET_ALL}")
                pass
        self.coords_to_send = []
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


YOLOV5_PATH = Path("/Users/taaha/codes/pytorch/ultralytics_yolov5/")
import sys
sys.path.append(str(YOLOV5_PATH))
import detect as dt
from utils.dataloaders import LoadWebcam
from utils.plots import Colors
from utils.augmentations import letterbox
weights = YOLOV5_PATH / 'runs' / 'train' / 'amin_ds' / 'weights' / 'best.pt'
data = YOLOV5_PATH / 'datasets' / 'amin_ds' / 'amin_ds.yaml'
device = torch.device('mps')

#Model
model = torch.hub.load(str(YOLOV5_PATH), 'custom', path=str(weights), source='local') # local repo

colors = Colors()

def add_dot_and_label(img, xc, yc, conf, cls, name, clr=None):
    if not clr: clr = colors(int(cls))
    label = f"{name}:{conf*100:.2f}%"

    im_h, im_w, channel = img.shape
    text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fontScale=(1.1e-3 * im_h), thickness=2)
    tx_width, tx_height = text_size[0][0], text_size[0][1]

    pt1 = (((xc+5) if (xc+10+tx_width < im_w) else (xc-tx_width-5)), ((yc-5) if (yc-10-tx_height > 0) else (yc+5+tx_height))) # bottom-left
    pt2 = (pt1[0]+10+tx_width, pt1[1]-10-tx_height) # top-right
    pt3 = (pt1[0], pt1[1]-10-tx_height) # top-left
    pt4 = (pt1[0]+10+tx_width, pt1[1]) # bottom-right
    # region = np.array([list(pt1),list(pt3),list(pt2),list(pt4)], dtype='int32')
    region = np.array([pt1,pt3,pt2,pt4], dtype='int32')    

    img = cv2.circle(img=img, center=(xc,yc), radius=10,color=clr, thickness=-1)
    img = cv2.rectangle(img=img, pt1=pt1, pt2=pt2, color=clr)
    img = cv2.fillPoly(img=img, pts=[region], color=clr)
    img = cv2.putText(img=img, text=label, org=(pt1[0]+5, pt1[1]-5), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=(1.1e-3 * im_h), color=(255,255,255), thickness=2)
    return img

def webcam(source=0):
    roscon = RosConnection()
    cap = cv2.VideoCapture(source)
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3) # set buffer size
    batch_size = 1
    images = []

    cls_count = {}
    for n in model.names:
        cls_count[n] = 0
    cls_coords = []
    im_count = 0
    while True:
        ret, img = cap.read()
        img = cv2.flip(img, 1)  # flip left-right
        if ret:
            images += [img]
        if (len(images) == batch_size) or (ret == False and len(images) > 0):
            for i in range(len(images)):
                t1 = dt.time_sync()
                
                pred = model(images[i])
                for idx, (xc, yc, w, h, conf, cls, name) in pred.pandas().xywh[0].iterrows():
                    images[i] = add_dot_and_label(images[i], int(xc), int(yc), conf, cls, name)
                    
                    if cls_count[name]:
                        cls_count[name] += 1
                    else:
                        cls_count[name] = 1
                    
                    if len(cls_coords) == 0:
                        cls_coords = [[name, xc, yc, 1]]
                    else: 
                        for i, (nm, xx, yy, n) in enumerate(cls_coords):
                            if abs(xx-xc) < 10 and abs(yy-yc) < 10: # 10 pixel error margin
                                if nm == name:
                                    cls_coords[i] = [nm,xc,yc,n+1]

                                    

                s = '['
                for n, c in cls_count.items():
                    if c > 0: 
                        s += f" {n}:{c},"
                        cls_count[n] = 0
                del_t = dt.time_sync() - t1
                s += f"] Done in:{del_t} sec"
                print(s)
                im_count+=1
                if im_count >= 5: # See for 5 frames
                    im_count = 0
                    coords = []
                    for i, (nm, xx, yy, n) in enumerate(cls_coords):
                        if n >= 4: # if 4 hits total in 5 frames
                            coords.append([nm, xx, yy])
                    if not roscon.busy:
                        rospy.loginfo(f"roscon no busy, sending:{coords}")
                        roscon.send_coords(coords=coords)
                    cls_coords = []

                # print fps on image
                images[i] = cv2.putText(images[i], f"FPS:{1/del_t:.2f}", (10, images[i].shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, (1.1e-3 * images[i].shape[0]), (255,255,255), thickness=3)
                cv2.imshow('video with bboxes', images[i])
            images = []
        if cv2.waitKey(1) == ord('q'):
            break  # q to quit
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=str, default=0, help='0 for webcam or address')
    try:
        webcam(parser.parse_args().source)
    except rospy.ROSInterruptException as e:
        raise e
