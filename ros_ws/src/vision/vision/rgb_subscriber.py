import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from ultralytics import YOLO
import matplotlib as plt
import cv2
import supervision as sv
import numpy as np

from tqdm import tqdm
from inference.models.yolo_world.yolo_world import YOLOWorld
from std_msgs.msg import Bool, Int32MultiArray, Float32

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

# __detector = "yolo-world"
# model = []

# if __detector == "yolo11":
#     model = YOLO("yolo11s.pt")
# elif __detector == "yolo-world":
model = YOLOWorld(model_id="yolo_world/l") 
classes = ["person","eyes"]
model.set_classes(classes)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('human_attention')
        self.get_logger().info("rgb subscriber node initialized.")
        self.br = CvBridge()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.rgb_subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_raw',
            self.rgb_callback,
            qos_policy)

        self.dist_subscription = self.create_subscription(
            Float32,
            '/distance',
            self.distance_callback,
            10)
        
        safety_distance_threshold = 3 

        self.publish_box = self.create_publisher(Int32MultiArray, '/box', 10)
        self.publish_attention = self.create_publisher(Bool, '/attention', 10)

        # self.tf_broadcaster = TransformBroadcaster(self)

        self.start_delay = 0
        self.sub_delay = 0
        self.not_seen = 0
        self.missed_frames= 5

        self.camera = {'fLeng': 2.87343,
                       'fDist': 0.6,
                       'fStop': 240,
                       'horAp': 5.76,
                       'verAp': 3.6,
                       'maxFOV': 150,
                       'frameW': 1280,
                       'frameH': 720}
        

    def distance_callback(self, msg):
        self.distance = msg.data
        self.get_logger().info(f"Distance received: {self.distance}")


    def eye_detection(self, img, x, y):

        x_min = int(x[0])
        y_min = int(y[0])
        x_max = int(x[1])
        y_max = int(y[1])

        image = img
        sub_image = image[y_min : y_max, x_min : x_max]

        # sub_results = model(sub_image)

        # detections = []

        # if __detector == "yolo11":
        #     detections = sv.Detections.from_ultralytics(sub_results[0]).with_nms(threshold=0.05)
        # elif __detector == "yolo-world":

        # detections = sv.Detections.from_inference(sub_results[0]).with_nms(threshold=0.05)

        sub_results = model.infer(sub_image, confidence=0.001)
        detections = sv.Detections.from_inference(sub_results).with_nms(threshold=0.05)

        sub_image = BOUNDING_BOX_ANNOTATOR.annotate(sub_image, detections)
        sub_image = LABEL_ANNOTATOR.annotate(sub_image, detections)

        cv2.imshow("Eyes", sub_image)
        
        
        sub_names = detections.data['class_name']


        for id in range(len(sub_names)):
            if sub_names[id] == "eyes":
                return(True)
        return(False)


    def rgb_callback(self, img):
        cv2Image = self.br.imgmsg_to_cv2(img)
        cv2Image = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2RGB)

        # results = model(cv2Image)

        # detections = []

        # if __detector == "yolo11":
        #     detections = sv.Detections.from_ultralytics(results[0]).with_nms(threshold=0.05)
        # elif __detector == "yolo-world":

        # detections = sv.Detections.from_inference(results[0]).with_nms(threshold=0.05)

        results = model.infer(cv2Image, confidence=0.8)
        detections = sv.Detections.from_inference(results).with_nms(threshold=0.5)


        names = detections.data['class_name']
        boxes = detections.xyxy.astype(int)

        if self.start_delay > 2:
            array_msg = [[0,0,0,0]]
            for id in range(len(names)):
                if names[id] != "person":
                    continue
                x = [int(boxes[id,0]), int(boxes[id,2])]
                y = [int(boxes[id,1]), int(boxes[id,3])]

                if self.distance <= safety_distance_threshold:
                    eye_detection = self.eye_detection(cv2Image, x, y)
                    if eye_detection == True:
                        self.publish_attention.publish(Bool(data=True))
                        self.not_seen=0
                else:
                    self.not_seen += 1
                    if self.not_seen >= self.missed_frames:
                        self.publish_attention.publish(Bool(data=False))
                        self.not_seen = self.missed_frames

                # array_msg = [array_msg, [x[0],x[1],y[0],y[1]]]
                array_msg = [x[0],x[1],y[0],y[1]]
                
                self.publish_box.publish(Int32MultiArray(data=array_msg))

            #self.publish_box.publish([x[0], y[0]],[x[1],y[1]])
        else:
            self.start_delay += 1


        annotated_image = cv2Image.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

        annotated_image = cv2.resize(annotated_image,(1280,720))

        # self.get_logger().info("Here it comes...")

        cv2.imshow("Person", annotated_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()