import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# from sensor_msgs.msg import Image  #which one to use for pub? 
from std_msgs.msg import Bool, Int32MultiArray, String

import cv2
import numpy as np

import supervision as sv
from ultralytics import YOLOWorld


BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

# model = YOLOWorld("../../../../data/hand_gestures_v6i.yolov5pytorch/runs/detect/train10/weights/last.pt") # might need to test the others 
model = YOLOWorld("/yolo/data/hand_gestures_v6i.yolov5pytorch/runs/detect/train10/weights/last.pt") # might need to test the others 

classes = ["stop", "Thumbs up"]
model.set_classes(classes)


class Gesture(Node):
    def __init__(self):
        super().__init__('gesture')
        self.get_logger().info("Gesture detection node initialized.")

        self.attention_subscriber = self.create_subscription(
            Bool,
            '/attention',
            self.attention_callback,
            10)

        self.gesture_publisher = self.create_publisher(String, '/command/gesture', 10) 

        self.cap = self.initialize_camera()

        # Stability mechanism
        self.last_published_gesture = None
        self.detection_counter = 0
        self.stability_threshold = 3  # tune if needed 
        self.current_gesture = None

    def initialize_camera(self): 
        #webcam
        cap = cv2.VideoCapture(0) # edit if another cam i needed 
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            exit()
        return cap

    @staticmethod
    def process_int_array_to_image(box_data, width=1280, height=720):
        array = np.array(box_data, dtype=np.int32)
        if array.size != width * height:
            return None  # Image size mismatch
        image = array.reshape((height, width))
        return np.clip(image, 0, 255).astype(np.uint8)


    def attention_callback(self, attention):
        # If attention is true, subscribe to the box topic
        if attention.data: 
            # self.get_logger().info("Attention detected")
            '''self.box_subscriber = self.create_subscription(
                Int32MultiArray,
                '/box',
                self.gesture_callback,
                qos_profile_sensor_data) '''
            self.gesture_callback()

    def gesture_callback(self): 
        # self.get_logger().info("Gesture callback triggered.")

        # Process image data
        # image = process_int_array_to_image(box.data)
        # if image is None:
        #     self.get_logger().error("Array size does not match image dimensions.")
        #     return
        
        ret, image = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image.")
            return 

        # Run detection
        detections = self.run_detection(image, threshold=0.1) # tune paramters
        detected_classes = detections.data['class_name']  

        # Publish and display annotated image
        try:
            if detected_classes in classes: #detected_classes and detected_classes[0] in classes: #look for the classes that is detected
                self.filter_and_publish(detected_classes[0]) 
                self.display_annotated_image(image, detections)
        except:
            self.get_logger().info(f"Too many gestrures detected")

    
    def run_detection(self, image, threshold=0.1):
        # self.get_logger().info("Running gesture detection model.")

        results = model.predict(image, conf=0.4)
        detections = sv.Detections.from_ultralytics(results[0]).with_nms(threshold)
        return detections


    def filter_and_publish(self, detected_gesture):
        if detected_gesture == self.current_gesture:
            self.detection_counter += 1
        else:
            self.detection_counter = 1  # Reset counter if a new gesture is detected
            self.current_gesture = detected_gesture

        # Publish gesture only if stable (and different from the last published one)
        if self.detection_counter >= self.stability_threshold: #and detected_gesture != self.last_published_gesture:
            # self.last_published_gesture = detected_gesture
            self.gesture_publisher.publish(String(data=detected_gesture))
            self.get_logger().info(f"Gesture detected - publishing: {detected_gesture}")


    def display_annotated_image(self, image, detections):
        annotated_image = image.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)
        
        cv2.imshow("Gesture Detection", annotated_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.get_logger().info("Gesture detection window closed.")


def main(args=None):
    rclpy.init(args=args)

    gesture_publisher = Gesture()
    rclpy.spin(gesture_publisher)

    gesture_publisher.destroy_node()
    rclpy.shutdown()

    # When everything is done, release the capture - not sure if it should be here 
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()