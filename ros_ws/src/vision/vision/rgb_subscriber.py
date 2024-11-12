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
from std_msgs.msg import Bool, Int32

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

# New branch BABY!

model = YOLO("yolo11s.pt")

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('human_attention')
        self.br = CvBridge()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.rgb_subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_raw',
            self.rgb_callback,
            qos_policy)
        
        self.publish_box = self.create_publisher(Int32, 'box', 10)
        self.publish_attention = self.create_publisher(Bool, 'attention', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_delay = 0
        self.sub_delay = 0

        self.camera = {'fLeng': 2.87343,
                       'fDist': 0.6,
                       'fStop': 240,
                       'horAp': 5.76,
                       'verAp': 3.6,
                       'maxFOV': 150,
                       'frameW': 1280,
                       'frameH': 720}
        

    def eye_detection(self, img, x, y):

        x_min = int(x[0])
        y_min = int(y[0])
        x_max = int(x[1])
        y_max = int(y[1])

        image = img
        sub_image = image[y_min : y_max, x_min : x_max]


        sub_results = model(sub_image)
        detections = sv.Detections.from_ultralytics(sub_results[0]).with_nms(threshold=0.5)
        sub_names = detections.data['class_name']

        attention = False

        if self.sub_delay > 2:
            for id in range(len(sub_names)):
                if sub_names[id] != "eyes":
                    continue
                attention = True
        else:
            self.sub_delay += 1


        return(attention)

        




    def rgb_callback(self, img):
        cv2Image = self.br.imgmsg_to_cv2(img)
        cv2Image = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2RGB)

        results = model(cv2Image)

        detections = sv.Detections.from_ultralytics(results[0]).with_nms(threshold=0.5)


        names = detections.data['class_name']
        boxes = detections.xyxy.astype(int)

        if self.start_delay > 2:
            print("## DETECTIONS ##")
            for id in range(len(names)):
                if names[id] != "person":
                    continue

                print(names[id])

                x = [int(boxes[id,0]), int(boxes[id,2])]
                y = [int(boxes[id,1]), int(boxes[id,3])]

                eye_detectioin = self.eye_detectioin(cv2Image, x, y)            
                self.publish_attention.publish(eye_detectioin)
                self.publish_box.publush(x, y)
        
    


        else:
            self.start_delay += 1


        annotated_image = cv2Image.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

        annotated_image = cv2.resize(annotated_image,(1280,720))

        cv2.imshow("YOLO11", annotated_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()