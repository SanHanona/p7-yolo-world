

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import String


# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('minimal_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         timer_period = 0.5  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = 'Hello World: %d' % self.i
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.data)
#         self.i += 1


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_publisher = MinimalPublisher()

#     rclpy.spin(minimal_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import matplotlib as plt
import cv2
import supervision as sv

from tqdm import tqdm
from inference.models.yolo_world.yolo_world import YOLOWorld

BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)

model = YOLOWorld(model_id="yolo_world/l")

classes = ["forklift","wheel","cardboard box","barrel","person"]
model.set_classes(classes)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.get_logger().info("Image subscriber node initialized.")
        self.br = CvBridge()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(
            Image,
            '/front_stereo_camera/left/image_raw',
            self.listener_callback,
            qos_policy)
        # self.subscription  # prevent unused variable warning


    def listener_callback(self, img):
        cv2Image = self.br.imgmsg_to_cv2(img)
        cv2Image = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2RGB)

        results = model.infer(cv2Image, confidence=0.2)
        detections = sv.Detections.from_inference(results).with_nms(threshold=0.50)

        annotated_image = cv2Image.copy()
        annotated_image = BOUNDING_BOX_ANNOTATOR.annotate(annotated_image, detections)
        annotated_image = LABEL_ANNOTATOR.annotate(annotated_image, detections)

        # self.get_logger().info("Generated goal pose: {0}".format(pose))
        # goal_msg.pose.pose.position.x = pose[0]
        # goal_msg.pose.pose.position.y = pose[1]
        # goal_msg.pose.pose.orientation.x = pose[2]
        # goal_msg.pose.pose.orientation.y = pose[3]
        # goal_msg.pose.pose.orientation.z = pose[4]
        # goal_msg.pose.pose.orientation.w = pose[5]

        cv2.imshow("image", annotated_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from rclpy.qos import qos_profile_sensor_data

# class MinimalSubscriber(Node):

#     def __init__(self):
#         super().__init__('image_subscriber')
#         qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
#                                           history=rclpy.qos.HistoryPolicy.KEEP_LAST,
#                                           depth=1)
        
#         self.subscription = self.create_subscription(
#             Image,
#             '/front_stereo_camera/left/image_raw',
#             self.listener_callback,
#             qos_policy)
        
#         # self.subscription  # prevent unused variable warning

#     def listener_callback(self):
#         print("In callback")

# def main(args=None):
#     rclpy.init(args=args)
#     minimal_subscriber = MinimalSubscriber()
#     rclpy.spin(minimal_subscriber)
#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_subscriber.destroy_node()
#     rclpy.shutdown()

#     if __name__ == '__main__':
#         main()