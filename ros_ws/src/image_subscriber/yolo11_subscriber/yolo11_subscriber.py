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

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

# New branch BABY!

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

        self.tf_broadcaster = TransformBroadcaster(self)

        self.depth = []
        self.start_delay = 0

        self.camera = {'fLeng': 2.87343,
                       'fDist': 0.6,
                       'fStop': 240,
                       'horAp': 5.76,
                       'verAp': 3.6,
                       'maxFOV': 150,
                       'frameW': 1280,
                       'frameH': 720}



    def rgb_callback(self, img):
        cv2Image = self.br.imgmsg_to_cv2(img)
        cv2Image = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2RGB)

        depth_rect = self.depth.copy()
        depth_rect = np.divide(depth_rect,10)

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
                # x_diff = int((x[1] - x[0])*0.2)
                # y_diff = int((y[1] - y[0])*0.2)

                depth_rect = cv2.rectangle(depth_rect, (x[0],y[0]), (x[1],y[1]), (255,0,255), 2)
                # depth_rect = cv2.rectangle(depth_rect, (x[0]+x_diff,y[0]+y_diff), (x[1]-x_diff,y[1]-y_diff), (255,0,0), 2)
                depth_cut = self.depth[y[0]:y[1], x[0]:x[1]]
                depth_sort = depth_cut.flatten()
                depth_sort.sort()

                obj_depth = np.median(depth_sort[0:int(len(depth_sort)*0.5)])
                obj_x = int((x[1]+x[0])/2)
                obj_y = int((y[1]+y[0])/2)

                depth_rect = cv2.putText(depth_rect, str(round(obj_depth,2)) + "m", (x[0]+5,y[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (255,0,0), 2, cv2.LINE_AA)
                depth_rect = cv2.circle(depth_rect, (obj_x,obj_y), 2, (255,0,255), 2)

                object_loc = self.locate_object(obj_x, obj_depth)

                self.handle_obj_tf(object_loc, id)
                
                print(object_loc)
        else:
            self.start_delay += 1


        annotated_image = cv2Image.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

        annotated_image = cv2.resize(annotated_image,(1280,720))

        cv2.imshow("YOLO11", annotated_image)
        try:
            cv2.imshow("Depth", depth_rect)
        except:
            print("No depth image!")
        cv2.waitKey(1)

    def depth_callback(self, img):
        cv2Image = self.br.imgmsg_to_cv2(img)
        self.depth = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2RGB)

    def locate_object(self, obj_x, obj_depth):
        cx = self.camera['frameW']/2

        Z = obj_depth
        X = ((obj_x-cx)*Z)/480
        # Y = ((obj_y-cy)*Z)/self.camera['fLeng']

        return [X,0.0,Z]

    def handle_obj_tf(self, loc, name):
        # grabbed from https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        child_name = 'object_link_' + str(name)
        t.child_frame_id = child_name

        t.transform.translation.x = float(loc[2])
        t.transform.translation.y = float(-loc[0])
        t.transform.translation.z = float(loc[1])

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()