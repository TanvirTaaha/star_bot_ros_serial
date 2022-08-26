import torch
from matplotlib import pyplot as plt
import cv2
from PIL import Image
from pathlib import Path

YOLOV5_PATH = Path("/Users/taaha/codes/pytorch/ultralytics_yolov5/")
weights = YOLOV5_PATH / 'runs' / 'train' / 'amin_ds' / 'weights' / 'best.pt'
data = YOLOV5_PATH / 'datasets' / 'amin_ds' / 'amin_ds.yaml'

#Model
model = torch.hub.load(str(YOLOV5_PATH), 'custom', path=str(weights), source='local') # local repo
# Images
img = cv2.imread('table.jpg')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY )
# Inference
results = model(img, size=640)  # includes NMS

# Results
results.print()
# results.show()  # or .show()

#results = results.xyxy[0]  # img1 predictions (tensor)
boxes = results.pandas().xyxy[0]  # img1 predictions (pandas)
print(boxes)

print('hhhhhhh')
for xc, yc, w, h, conf, cls, name in results.pandas().xywh.iterrows():
    print(xc, yc, w, h, conf, cls, name)
    

