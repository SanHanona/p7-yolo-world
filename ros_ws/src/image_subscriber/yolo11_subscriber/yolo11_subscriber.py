import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

from ultralytics import YOLO
import matplotlib as plt
import cv2
import supervision as sv

from tqdm import tqdm
from inference.models.yolo_world.yolo_world import YOLOWorld

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

model = YOLO("yolo11s.pt")

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('yolo11_subscriber')
        self.br = CvBridge()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.rgb_subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_raw',
            self.rgb_callback,
            qos_policy)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/front_cam/depth',
            self.depth_callback,
            qos_policy)
        # self.subscription  # prevent unused variable warning

        self.depth = []



    def rgb_callback(self, img):
        cv2Image = self.br.imgmsg_to_cv2(img)
        cv2Image = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2RGB)

        results = model(cv2Image)

        detections = sv.Detections.from_ultralytics(results[0]).with_nms(threshold=0.5)

        # print(detections.xyxy)
        # print(detections.data['class_name'])

        names = detections.data['class_name']
        boxes = detections.xyxy

        print("## DETECTIONS ##")
        for id in range(len(names)):
            print(names[id])
            print(boxes[id])

        annotated_image = cv2Image.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

        annotated_image = cv2.resize(annotated_image,(1280,720))

        cv2.imshow("YOLO11", annotated_image)
        cv2.imshow("Depth", cv2Image)
        cv2.waitKey(1)
    
    def depth_callback(self, img):
        cv2Image = self.br.imgmsg_to_cv2(img)
        self.depth = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2RGB)
        # cv2.imshow("Depth", cv2Image)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()