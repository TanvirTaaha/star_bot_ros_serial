import cv2
import cv2.aruco as aruco
import numpy as np
import os
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



def main():
    addr = "https://192.168.43.1:8080/video"
    cap = cv2.VideoCapture(addr)
    CORNERS = {
        23 : 1,
        45 : 2,
        65 : 3,
        69 : 4}
    while True:
        # try:
        ret, frame = cap.read()
        # ret = True
        # frame = cv2.imread('../tags.jpg')
        if ret:
            original_frame = frame.copy()
            aruco_found = findArucoMarkers(frame)
        
            if aruco_found:
                # print(f"aruco_found:{aruco_found}")
                try:
                    rect = get_ref_rectangle(aruco_found[0], aruco_found[1])
                except KeyError as e:
                    print(f"{Fore.RED}{e}{Style.RESET_ALL}")
                    continue
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
                    cv2.imshow('warped', warped)

            cv2.imshow("Image", frame)
            if cv2.waitKey(1) == ord('q'):
                cap.release()
                cv2.destroyAllWindows()


        # except Exception as e:
        #     print(f"{Fore.RED}{e}{Style.RESET_ALL}")
        #     cv2.destroyAllWindows()


if __name__ == '__main__':
    main()