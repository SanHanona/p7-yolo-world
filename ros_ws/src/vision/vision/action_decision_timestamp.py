import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32

class ActionDecision(Node):
    def __init__(self):
        super().__init__('action_decision')
        self.get_logger().info("Action decision node initialized.")

        # Subscriptions
        self.attention_subscriber = self.create_subscription(
            Bool,
            '/attention',
            self.attention_callback,
            10)

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

        # Publisher
        self.publish_command = self.create_publisher(String, '/command/action', 10)

        # State Variables
        self.attention = False

        self.last_language_command = None
        self.last_language_timestamp = None

        self.last_gesture_command = None
        self.last_gesture_timestamp = None

        self.safety_distance = None
        self.last_distance_timestamp = None

        self.command_window = 5.0  # Command validity in seconds

        self.timer = self.create_timer(1, self.timer_callback)

    def attention_callback(self, msg):
        # If attention is true
        if msg.data: 
            self.attention = msg.data

    def language_callback(self, msg):
        self.last_language_command = msg.data()
        self.last_language_timestamp = self.get_clock().now()
        self.get_logger().info(f"Language received: {self.last_language_command}")

    def gesture_callback(self, msg):
        self.last_gesture_command = msg.data()
        self.last_gesture_timestamp = self.get_clock().now()
        self.get_logger().info(f"Gesture received: {self.last_gesture_command}")

    def distance_callback(self, msg):
        self.safety_distance = msg.data
        self.last_distance_timestamp = self.get_clock().now()
        self.get_logger().info(f"Distance received: {self.safety_distance}")

    def timer_callback(self):
        # Get current time
        now = self.get_clock().now()

        # Determine active commands based on timestamps
        active_language = self.last_language_command if self.is_recent(self.last_language_timestamp, now) else None
        active_gesture = self.last_gesture_command if self.is_recent(self.last_gesture_timestamp, now) else None

        active_distance = self.is_distance_within_range()

        # Combine commands and prioritize
        self.decide_action(active_language, active_gesture, active_distance)

    def is_recent(self, timestamp, current_time):
        """Check if the timestamp is within the command window."""
        if timestamp is None:
            return False
        return (current_time - timestamp).nanoseconds / 1e9 <= self.command_window

    def is_distance_within_range(self):
        """Check if the distance is within the stopping range."""
        if self.safety_distance is None or self.last_distance_timestamp is None:
            return False
        return self.safety_distance <= 3 # change if needed 

    def decide_action(self, language, gesture, safety_distance):
        """Decide on the action based on the hierarchy."""
        if "stop" in {gesture, language}:
            self.stop()
        elif "wait" in {gesture, language}:
            self.wait()
        elif "pass" in {gesture, language}:
            self.pass_action()
        elif safety_distance:
            self.stop()
            self.get_logger().info("Default stop due to safety distance.")
            if not self.attention: 
                self.get_attention():
        else:
            # self.stop()
            self.get_logger().info("No valid commands or conditions met.")

    def get_attention(self): 
        self.get_logger().info("Trying to get attention of worker")
        # Implement LED blink or sound  

    # Jonas do you thing in the functions below 
    def stop(self):
        self.get_logger().info("Action: Stop!")
        self.publish_command.publish(String(data="stop"))

    def wait(self):
        self.get_logger().info("Action: Wait...")
        self.publish_command.publish(String(data="wait"))

    def pass_action(self):
        self.get_logger().info("Action: Pass.")
        self.publish_command.publish(String(data="pass"))

def main(args=None):
    rclpy.init(args=args)
    action_decision = ActionDecision()
    rclpy.spin(action_decision)
    action_decision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
