import argparse
import torch
import cv2
import cv2.aruco as aruco
from pathlib import Path
import numpy as np

import rospy
from std_msgs.msg import Int16MultiArray

import math
from colorama import Fore, Style


def findArucoMarkers(img, markerSize=4, totalMarkers=250, draw=True):
    ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
    }
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("Grey", imgGray)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs = []
    ids = []
    bboxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)
    # print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return (ids, bboxs) if not np.logical_or(ids == None, bboxs == None).any() else None


def get_ref_rectangle(ids, boxs):
    # print(f"boxs:{boxs}")
    rect = [[],[],[],[]]
    CORNERS = {
        23 : 1,
        45 : 2,
        65 : 3,
        69 : 4}
    for i, id in enumerate(ids):
        id = id[0]
        # print(f"i:{i}")
        x_sum = boxs[i][0][0][0]+ boxs[i][0][1][0]+ boxs[i][0][2][0]+ boxs[i][0][3][0]
        y_sum = boxs[i][0][0][1]+ boxs[i][0][1][1]+ boxs[i][0][2][1]+ boxs[i][0][3][1]

        x_centerPixel = x_sum / 4
        y_centerPixel = y_sum / 4
        rect[CORNERS[id]-1] = (int(x_centerPixel), int(y_centerPixel))

    # if sum(1 if len(x) == 2 else 0 for x in rect) == 4:
    #     rect = sorted(rect, key=lambda x: x[1]) # sort by x value
    #     (rect[0], rect[1]) == (rect[1], rect[0]) if rect[1][0] < rect[0][0] else (rect[0], rect[1])
    #     (rect[2], rect[3]) == (rect[3], rect[2]) if rect[3][0] > rect[2][0] else (rect[2], rect[3])
        
    return rect

def four_point_transform(image, rect):
    (tl, tr, br, bl) = rect
    # print((tl, tr, br, bl))
    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
		[0, 0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]], dtype = "float32")
    rect = np.array(rect, dtype='float32')
    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    # return the warped image
    return warped

# def warp_cut(img, pts1, width=40, height=40):
#     pts2 = np.array([[0, 0], [width, 0], [width, height], [0, height]])


def get_warped(frame):
    warped = None
    found = False
    CORNERS = {
        23 : 1,
        45 : 2,
        65 : 3,
        69 : 4}
    original_frame = frame.copy()
    aruco_found = findArucoMarkers(frame)

    if aruco_found:
        # print(f"aruco_found:{aruco_found}")
        try:
            rect = get_ref_rectangle(aruco_found[0], aruco_found[1])
        except KeyError as e:
            print(f"{Fore.RED}{e}{Style.RESET_ALL}")
            return None
        print(f"rect:{rect}")
        if sum([1 if len(x)==2 else 0 for x in rect]) == 4:
        
            for i, p in enumerate(rect):
                # print(f"p:{p}, type:{type(p)}")
                cv2.putText(frame, str(i+1), (p[0],p[1]), cv2.FONT_HERSHEY_SIMPLEX, (1.1e-3 * frame.shape[0]), (0,0,255), 3)
        
            overlay = frame.copy()
            # cv2.rectangle(img=overlay, pt1=rect[0], pt2=rect[2], color=(128, 128, 0), thickness=-1)
            
            cv2.fillPoly(img=overlay, pts=np.array([rect]), color=(128, 128, 0))
            alpha = 0.4
            frame = cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0)
    
            # print(rect)
            warped = four_point_transform(original_frame, rect=rect)
            # cv2.imshow('warped', warped)
            found = True
            
        cv2.imshow("Feed", frame)
    return found, warped




class InverseKinematics:
    def __init__(self) -> None:

        self.pub = rospy.Publisher('cmd_angle', Int16MultiArray, queue_size=10)
        rospy.init_node('angle_controller_ik', anonymous=True, log_level=rospy.DEBUG)
        self.rate = rospy.Rate(1)
        self.busy = False
        self.coords_to_send = []
        self.origin = (27.5, -20) 
        self.HEIGHT = 16 # 16 cm
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
        rospy.loginfo(f'before for loop{coords}')
        for i, (nm, x, y) in enumerate(self.coords_to_send):
            msg = Int16MultiArray()
            rospy.loginfo('before try')
            try:
                msg.data = [self.NAMES[nm], *self.calculate_base_rotation(nm, x, y)]
                msg.data = [int(x) for x in msg.data]
                self.pub.publish(msg)
                self.rate.sleep()
                self.pub.publish(msg)
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
        base_angle = math.degrees(math.atan((x-self.origin[0]) / (y-self.origin[1])))
        r = math.sqrt((self.origin[0]-x)**2 + (self.origin[1]-y)**2)
        r = r - 16 # 16 cm back from actual object
        z = -self.HEIGHT #Depth from base
        try:
            shoulder_angle, elbow_angle = self.inv_kinematics(r, z)
            pitch_angle = 30
            return base_angle, shoulder_angle, elbow_angle, pitch_angle
        except TypeError as e:
            print(f"{Fore.LIGHTRED_EX}In CALC_BASE_ROTATION:{e}{Style.RESET_ALL}")
            return None
        


            
    def inv_kinematics(self, r, z, l1=20, l2=22):
        if not (-1 <= (r**2+z**2-l1**2-l2**2)/(2*l1*l2) <= 1):
            # domain test of acos
            # also limit that is possible for the two links to reach
            rospy.logerr(f'Domain error')
            return

        # Equations for Inverse kinematics
        phi_2 = (math.acos((r**2+z**2-l1**2-l2**2)/(2*l1*l2)))  # eqn 2
        phi_1 = math.atan2(z, r) + math.atan2(l2*math.sin(phi_2), (l1 + l2*math.cos(phi_2))) # eqn 3
        # phi_2 = -phi_2
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

def webcam(source=0, tag_width = 55, tag_height = 30):
    roscon = InverseKinematics()
    invert = False
    if len(source) == 1:
        source = int(source)
        if source == 0: # if webcam then flip
            invert = True
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3) # set buffer size
    batch_size = 1
    images = []

    cls_count = {}
    for n in model.names:
        cls_count[n] = 0
    cls_coords = []
    im_count = 0
    while not rospy.is_shutdown():
        ret, img = cap.read()
        if invert: # flip right left
            img = cv2.flip(img, 1)  # flip left-right
        if ret:
            images += [img]
        if (len(images) == batch_size) or (ret == False and len(images) > 0):
            for i in range(len(images)):
                t1 = dt.time_sync()
                try:
                    found, warped = get_warped(images[i])
                except TypeError as e:
                    print(f"{Fore.LIGHTRED_EX}{e}{Style.RESET_ALL}")
                    continue
                
                if found:
                    pred = model(warped)
                    for idx, (xc, yc, w, h, conf, cls, name) in pred.pandas().xywh[0].iterrows():
                        warped = add_dot_and_label(warped, int(xc), int(yc), conf, cls, name)
                        
                        print(f"warped Height:{warped.shape[0]}, Width:{warped.shape[1]}")
                        warp_h, warp_w, _ = warped.shape
                        xc = xc / warp_w * tag_width
                        yc = yc / warp_h * tag_height

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
                            print(f"after 5 frames{(nm, xx, yy, n)}")
                            if n >= 4: # if 4 hits total in 5 frames
                                coords.append([nm, xx, yy])
                        if not roscon.busy:
                            rospy.loginfo(f"roscon no busy, sending:{coords}")
                            roscon.send_coords(coords=coords)
                        cls_coords = []

                    # print fps on image
                    warped = cv2.putText(warped, f"FPS:{1/del_t:.2f}", (10, images[i].shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, (1.1e-3 * warped.shape[0]), (255,255,255), thickness=3)
                    cv2.imshow('video with bboxes', warped)
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
