import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

import cv2
import numpy as np
import os

import supervision as sv
from ultralytics import YOLOWorld

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

model = YOLOWorld("/yolo/data/hand_gestures_v6i.yolov5pytorch/runs/detect/train10/weights/last.pt")

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
        self.stability_threshold = 3
        self.current_gesture = None

        # Setup results folder for saving CSVs
        self.results_folder = "gesture_test"
        self.initialize_results_folder()

        # Determine the CSV file for this run
        self.csv_file = self.get_next_log_file_name()

    def initialize_results_folder(self):
        """Create the results folder if it does not exist."""
        if not os.path.exists(self.results_folder):
            os.makedirs(self.results_folder)
            self.get_logger().info(f"Created folder: {self.results_folder}")

    def get_next_log_file_name(self):
        """Generate the next available log file name with an incrementing number."""
        files = os.listdir(self.results_folder)
        file_numbers = [
            int(f.split('.')[0].replace('gesture_test', ''))
            for f in files if f.startswith("gesture_test") and f.endswith(".csv")
        ]
        next_number = max(file_numbers, default=0) + 1
        return os.path.join(self.results_folder, f"gesture_test{next_number}.csv")

    def append_to_results(self, test_value):
        """Save a single test result immediately to the CSV file."""
        try:
            with open(self.csv_file, 'a') as f:
                f.write(f"{test_value}\n")
            self.get_logger().info(f"Appended result to {self.csv_file}: {test_value}")
        except Exception as e:
            self.get_logger().error(f"Failed to append result: {e}")

    def initialize_camera(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            exit()
        return cap

    @staticmethod
    def process_int_array_to_image(box_data, width=1280, height=720):
        array = np.array(box_data, dtype=np.int32)
        if array.size != width * height:
            return None
        image = array.reshape((height, width))
        return np.clip(image, 0, 255).astype(np.uint8)

    def attention_callback(self, attention):
        if attention.data:
            self.gesture_callback()

    def gesture_callback(self):
        ret, image = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image.")
            return

        detections = self.run_detection(image, threshold=0.1)
        detected_classes = detections.data['class_name']

        try:
            if detected_classes in classes:
                self.filter_and_publish(detected_classes[0])
                self.display_annotated_image(image, detections)
            else: 
                self.append_to_results(detected_classes)
                print(detected_classes)
        except:
            error_msg = "Too many gestures detected"
            self.get_logger().info(error_msg)
            self.append_to_results(error_msg)

    def run_detection(self, image, threshold=0.1):
        results = model.predict(image, conf=0.4)
        detections = sv.Detections.from_ultralytics(results[0]).with_nms(threshold)
        return detections

    def filter_and_publish(self, detected_gesture):
        if detected_gesture == self.current_gesture:
            self.detection_counter += 1
        else:
            self.detection_counter = 1
            self.current_gesture = detected_gesture

        if self.detection_counter >= self.stability_threshold:
            self.gesture_publisher.publish(String(data=detected_gesture))
            self.append_to_results(detected_gesture)
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
    try:
        rclpy.spin(gesture_publisher)
    finally:
        gesture_publisher.destroy_node()
        rclpy.shutdown()
        gesture_publisher.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
