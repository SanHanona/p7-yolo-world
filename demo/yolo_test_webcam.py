import cv2
import supervision as sv

from tqdm import tqdm
from inference.models.yolo_world.yolo_world import YOLOWorld

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

model = YOLOWorld(model_id="yolo_world/s") #can be l, m, s (large, medium, small)

classes = ["human"]
model.set_classes(classes)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

frame_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

<<<<<<< HEAD
    results = model.infer(frame, confidence=0.002)
=======
<<<<<<< HEAD
    results = model.infer(frame, confidence=0.002, )
=======
    results = model.infer(frame, confidence=0.002)
>>>>>>> origin/fine_tune_hand_guestures
>>>>>>> db6c6324138a7ba00265d32bf682105d983d09ab
    detections = sv.Detections.from_inference(results).with_nms(threshold=0.1)

    annotated_image = frame.copy()
    annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
    annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

    # Display the resulting frame
    cv2.imshow('Annotated Video', annotated_image)

    # Press 'q' to exit the video stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()



