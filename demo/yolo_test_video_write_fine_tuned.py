import cv2
import supervision as sv

from tqdm import tqdm
# from inference.models.yolo_world.yolo_world import YOLOWorld

from ultralytics import YOLOWorld


BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

model = YOLOWorld("../tools/work_dirs/custom_fine_tune_2/epoch_15.pth")

SOURCE_VIDEO_PATH = f"data/slow_traffic_small.mp4"
video_capture = cv2.VideoCapture(SOURCE_VIDEO_PATH)

classes = ["car", "person"]
model.set_classes(classes)

## Define video capture and check if it's opened successfully
if not video_capture.isOpened():
    print("Error: Could not open video.")
else:
    # Get the video's width, height, and frames per second
    frame_width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(video_capture.get(cv2.CAP_PROP_FPS))
    
    # Define the codec and create a VideoWriter object for MP4 output
    output_filename = 'data/annotated_video.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4
    out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

    while True:
        ret, frame = video_capture.read()
        if not ret:
            break

        # Perform inference and annotate frame
        results = model.predict(frame, conf=0.1, iou=0.1) #conf=0.25, iou=0.45
        detections = sv.Detections.from_ultralytics(results[0]).with_nms(threshold=0.1)

        annotated_image = frame.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

        # Write the annotated frame to the video file
        out.write(annotated_image)

    # Release video resources
    video_capture.release()
    out.release()
    cv2.destroyAllWindows()

    print(f"Video saved as {output_filename}")


