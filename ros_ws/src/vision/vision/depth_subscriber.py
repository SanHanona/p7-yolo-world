import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Int32MultiArray, Float32

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.get_logger().info("Depth subscriber node initialized.")
        self.br = CvBridge()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.depth_subscription = self.create_subscription(
            Image,
            '/front_cam/depth',
            self.depth_callback,
            qos_policy)
        
        self.box_subscription = self.create_subscription(
            Int32MultiArray,
            '/box',
            self.box_callback,
            10)
        
        self.publish_distance = self.create_publisher(Float32, '/distance', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.depth = []
        self.boxes = []

        self.d_flag = False
        self.b_flag = False

        self.camera = {'fLeng': 2.87343,
                       'fDist': 0.6,
                       'fStop': 240,
                       'horAp': 5.76,
                       'verAp': 3.6,
                       'maxFOV': 150,
                       'frameW': 1280,
                       'frameH': 720}
        
        
    def depth_callback(self, img):
        cv2Image = self.br.imgmsg_to_cv2(img)
        self.depth = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2RGB)

        self.d_flag = True

        self.combine()


    def box_callback(self, box_msg):
        self.boxes = box_msg.data
        print(self.boxes)
        
        self.b_flag = True

        self.combine()


    def combine(self):
        if self.d_flag and self.b_flag:
            # self.get_logger().info("Here it comes...")
            self.b_flag = False
            self.d_flag = False
            self.locate_object()


    def locate_object(self):

        depth_rect = self.depth.copy()
        depth_rect = np.divide(depth_rect,20)

        
        for id in range(len(self.boxes)):

            box = self.boxes

            x = [box[0], box[1]]
            y = [box[2], box[3]]

            depth_rect = cv2.rectangle(depth_rect, (x[0],y[0]), (x[1],y[1]), (255,0,255), 2)
            depth_cut = self.depth[y[0]:y[1], x[0]:x[1]]
            depth_sort = depth_cut.flatten()
            depth_sort.sort()

            obj_depth = np.median(depth_sort[0:int(len(depth_sort)*0.5)])
            obj_x = int((x[1]+x[0])/2)
            obj_y = int((y[1]+y[0])/2)

            depth_rect = cv2.putText(depth_rect, str(round(obj_depth,2)) + "m", (x[0]+5,y[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 
                1, (255,0,0), 2, cv2.LINE_AA)
            depth_rect = cv2.circle(depth_rect, (obj_x,obj_y), 2, (255,0,255), 2)

            # object_loc = self.locate_object(obj_x, obj_depth)
            cx = self.camera['frameW']/2
            Z = obj_depth
            X = ((obj_x-cx)*Z)/480

            self.handle_obj_tf([X,0.0,Z], id)

            cx = self.camera['frameW']/2

            Z = obj_depth
            X = ((obj_x-cx)*Z)/(self.camera['fStop']*2)
            # Y = ((obj_y-cy)*Z)/self.camera['fLeng']

            self.publish_distance.publish(Float32(data=float(obj_depth)))

            cv2.imshow("Depth", depth_rect)
            cv2.waitKey(1)

    
    
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
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()