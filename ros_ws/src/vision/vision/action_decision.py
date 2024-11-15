import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

# ros2 topic pub -1 /command/gesture std_msgs/msg/String "{data: 'stop'}"


class ActionDecision(Node):
    def __init__(self):
        super().__init__('action_decision')
        self.get_logger().info("Action decision node initialized.")

        self.lang_subscription = self.create_subscription(
            String,
            '/command/language',
            self.language_callback,
            10)
        
        self.gest_subscription = self.create_subscription(
            String,
            '/command/gesture',
            self.gesture_callback,
            10)
        
        self.dist_subscription = self.create_subscription(
            Float32,
            '/distance',
            self.distance_callback,
            10)

        self.publish_command = self.create_publisher(String, '/command/action', 10)

        self.timer = self.create_timer(1, self.timer_callback)

        self.language = ""
        self.gesture = ""
        self.distance = 0

        self.language_timer = 0
        self.gesture_timer = 0
        self.distance_timer = 0
        
        
    def language_callback(self, msg):
        self.language = msg.data
        self.language_timer = 5
        self.get_logger().info('Language callback: ' + self.language)

    def gesture_callback(self, msg):
        self.gesture = msg.data
        self.gesture_timer = 5
        self.get_logger().info('gesture callback: ' + self.gesture)

    def distance_callback(self, msg):
        self.distance = msg.data
        self.distance_timer = 5
        self.get_logger().info('distance callback: ' + str(self.distance))

    def timer_callback(self):
        # self.get_logger().info('Timering')
        if self.gesture_timer > 0 and self.gesture == 'stop':
            self.gesture_timer -= 1
            self.stop()
        elif self.gesture_timer > 0 and self.gesture == 'wait':
            self.gesture_timer -= 1
            self.wait()
        elif self.gesture_timer > 0 and self.gesture == 'something else':
            self.gesture_timer -= 1
            self.something_else()


    def stop(self):
        self.get_logger().info('Stop!')

    def wait(self):
        self.get_logger().info('Wait...')

    def something_else(self):
        self.get_logger().info('Something else?')


def main(args=None):
    rclpy.init(args=args)
    action_decision = ActionDecision()
    rclpy.spin(action_decision)
    action_decision.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()