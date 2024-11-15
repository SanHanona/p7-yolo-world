import cv2
import supervision as sv

from tqdm import tqdm
from ultralytics import YOLOWorld

import os
HOME = os.getcwd()
print(HOME)

SOURCE_IMAGE_PATH = f"data/ining-and-complaining-standing-disappointed-against-white-background-2GGMYFA_jpg.rf.35e13df2553e5f2450f0bea09ee3a9b5.jpg"

model = YOLOWorld("../data/hand_gestures_v6i.yolov5pytorch/runs/detect/train13/weights/last.pt") 

classes = ["stop", "Thumbs up"]
model.set_classes(classes)

image = cv2.imread(SOURCE_IMAGE_PATH)
results = model.predict(image, conf=0.1, iou=0.1)
detections = sv.Detections.from_ultralytics(results[0]).with_nms(threshold=0.1)

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

annotated_image = image.copy()
annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)
sv.plot_image(annotated_image, (10, 10))