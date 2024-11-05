from ultralytics import YOLO
import cv2
import supervision as sv

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK, )

model = YOLO("yolo11m.pt")

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()


frame_count = 0
while True:

    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    detections = sv.Detections.from_ultralytics(results[0]).with_nms(threshold=0.3)

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


