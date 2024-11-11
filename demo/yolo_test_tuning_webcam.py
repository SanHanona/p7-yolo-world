import cv2
import supervision as sv
import time 


from tqdm import tqdm
from inference.models.yolo_world.yolo_world import YOLOWorld


BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)


model = YOLOWorld(model_id="yolo_world/s") #can be l, m, s (large, medium, small)
classes = ['person']
model.set_classes(classes)

sub_model = YOLOWorld(model_id="yolo_world/s") #can be l, m, s (large, medium, small)
sub_classes = ["eyes"]
sub_model.set_classes(sub_classes)


cap = cv2.VideoCapture(0)
avg_confidence=0
avg_fps=0

t1 = 0

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

frame_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.infer(frame, confidence=0.2)
    detections = sv.Detections.from_inference(results).with_nms(threshold=0.1)

    
    for person_id in range(len(detections.class_id)):
        class object_param:
            x_min = int(detections.xyxy[person_id][0])
            y_min = int(detections.xyxy[person_id][1])
            x_max = int(detections.xyxy[person_id][2])
            y_max = int(detections.xyxy[person_id][3])
        
        sub_frame = frame[object_param.y_min : object_param.y_max, object_param.x_min : object_param.x_max,]

        sub_results = sub_model.infer(sub_frame, confidence=0.008)
        sub_detections = sv.Detections.from_inference(sub_results).with_nms(threshold=0.005)

        
        sub_labels = [
            f"{sub_classes[class_id]} {confidence:0.3f}"
            for class_id, confidence
            in zip(sub_detections.class_id, sub_detections.confidence)]

        sub_annotated_image = sub_frame.copy()
        sub_annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(sub_annotated_image, sub_detections)
        sub_annotated_image = LABEL_ANNOTATOR.annotate(sub_annotated_image, sub_detections, labels=sub_labels)

        try:
            if len(sub_detections.class_id) >= 1:
                print("faceing the camera", person_id)
        except:
            pass


        cv2.imshow(f"person {str(person_id)}", sub_annotated_image)
        


    labels = [
        f"{classes[class_id]} {confidence:0.3f}"
        for class_id, confidence
        in zip(detections.class_id, detections.confidence)]
    

    

    annotated_image = frame.copy()
    annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
    annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections, labels=labels)


    #calculate the frame-rate    
    t2 = time.time()
    fps = 1/(t2-t1)
    t1 = t2

    avg_fps=(avg_fps+fps)/2

    try:
        avg_confidence = (avg_confidence + detections.confidence[0])/2
    except:
        pass
    

    #convert the frame-rate into an integer and then to a string, so it can be displayed on the image
    fps=int(fps)
    fps=str(fps)

    cv2.putText(annotated_image, fps, (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 2, (100, 255, 0), 3, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow('Annotated Video', annotated_image)

    # Press 'q' to exit the video stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
#print(detections.confidence) #get the confidence level for persons in the frame
#print(detections.xyxy[0]) #get the xy_min and xy_max of the bounding boxes in the frame
#print(results.predictions[0]) #get the xy_min and width and height of the bounding boxes in the frame using the results instead of detection
#print("average confidence", avg_confidence)
#print("average framerate", avg_fps)
cap.release()
cv2.destroyAllWindows()
''