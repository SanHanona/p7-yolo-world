import cv2
import supervision as sv

from tqdm import tqdm
from inference.models.yolo_world.yolo_world import YOLOWorld

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

SOURCE_VIDEO_PATH = f"data/simplescreenrecorder-2024-10-10_14.57.30.mp4"
video_capture = cv2.VideoCapture(SOURCE_VIDEO_PATH)

model = YOLOWorld(model_id="yolo_world/l")

classes = ["chair","table","shelf","person"]
model.set_classes(classes)

if not video_capture.isOpened():
    print("Error: Could not open video.")
else:
    while True:
        ret, frame = video_capture.read()
        if not ret:
            break

        results = model.infer(frame, confidence=0.2)
        detections = sv.Detections.from_inference(results).with_nms(threshold=0.85)

        annotated_image = frame.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

        # Display the resulting frame
        cv2.imshow('Annotated Video', annotated_image)

        # Press 'q' to exit the video stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()



