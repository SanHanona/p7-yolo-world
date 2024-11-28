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

        # self.dist_subscription = self.create_subscription(
        #     Float32,
        #     '/distance',
        #     self.distance_callback,
        #     10)

        # Publisher
        self.publish_command = self.create_publisher(String, '/command/action', 10)

        # State Variables
        self.last_attention_command = None
        self.last_attention_timestamp = None

        self.last_language_command = None
        self.last_language_timestamp = None

        self.last_gesture_command = None
        self.last_gesture_timestamp = None

        self.pass_state = False
        self.pass_command_count = 0  

        # self.safety_distance = None
        # self.last_distance_timestamp = None

        self.command_window = 3.0  # Command validity in seconds
        self.wait_time_after_attention_lost = 3.0  # Wait time before deactivating pass_state

        self.timer = self.create_timer(1, self.timer_callback)

    def attention_callback(self, msg):
        self.last_attention_command = msg.data  
        self.last_attention_timestamp = self.get_clock().now()
        self.get_logger().info(f"Attention state updated: {self.last_attention_command}")

    def language_callback(self, msg):
        self.last_language_command = msg.data
        self.last_language_timestamp = self.get_clock().now()
        self.get_logger().info(f"Language received: {self.last_language_command}")

    def gesture_callback(self, msg):
        self.last_gesture_command = msg.data
        self.last_gesture_timestamp = self.get_clock().now()
        self.get_logger().info(f"Gesture received: {self.last_gesture_command}")


    # def distance_callback(self, msg):
    #     self.safety_distance = msg.data
    #     self.last_distance_timestamp = self.get_clock().now()
    #     self.get_logger().info(f"Distance received: {self.safety_distance}")
    # def is_distance_within_range(self):
    #     """Check if the distance is within the stopping range."""
    #     if self.safety_distance is None or self.last_distance_timestamp is None:
    #         return False
    #     return self.safety_distance <= 3 # change if needed 


    def timer_callback(self):
        # Get current time
        now = self.get_clock().now()

        # Determine active commands based on timestamps
        active_language = self.last_language_command if self.is_recent(self.last_language_timestamp, now) else None
        active_gesture = self.last_gesture_command if self.is_recent(self.last_gesture_timestamp, now) else None

        active_attention = self.last_attention_command if self.is_recent(self.last_attention_timestamp, now) else None

        # Handle pass state logic
        self.handle_pass_state(active_language, active_gesture, active_attention, now)

        # Combine commands and prioritize
        self.decide_action(active_language, active_gesture, active_attention)

        # active_distance = self.is_distance_within_range()

    def is_recent(self, timestamp, current_time):
        """Check if the timestamp is within the command window."""
        if timestamp is None:
            return False
        return (current_time - timestamp).nanoseconds / 1e9 <= self.command_window

    def handle_pass_state(self, language, gesture, attention, current_time):
        """Manage the pass_state based on commands and attention."""
        # current issue - attention is stored for command window time, which is the same as time_since_last_attention
        if self.pass_state and attention is None:  
            time_since_last_attention = (
                (current_time - self.last_attention_timestamp).nanoseconds / 1e9
                if self.last_attention_timestamp else float('inf')
            )
            self.get_logger().info(f"pass state {self.pass_state} - attention is {attention}")
            if time_since_last_attention > self.wait_time_after_attention_lost:
                self.pass_state = False
                self.get_logger().info("Pass state deactivated due to attention loss.")
        
        if language == "pass" or gesture == "pass":
            self.pass_command_count += 1
            if self.pass_command_count >= 3:
                self.pass_state = True
                self.get_logger().info("Pass state activated.")
        else:
            self.pass_command_count = 0

    def decide_action(self, language, gesture, attention):
        """Decide on the action based on the hierarchy."""
        if attention or self.pass_state:
            if "stop" in {gesture, language}:
                self.stop_action()
            elif "wait" in {gesture, language}:
                self.wait_action()
            elif "pass" in {gesture, language} or attention in [True, False]:  # Handles attention True/False explicitly
                self.publish_action("pass")
            else:
                self.get_logger().info("No valid commands or conditions met.")
        elif attention == False:
            self.stop_action()
            self.get_logger().info("No valid commands or conditions met.")
            self.get_attention()
        else: 
            self.get_logger().info("action handler passive")

    def get_attention(self): 
        self.get_logger().info("Trying to get attention of worker")
        # Implement LED blink or sound  

    # Jonas do you thing in the functions below 
    def stop_action(self):
        self.get_logger().info("Action: Stop!")
        self.publish_command.publish(String(data="stop"))

    def wait_action(self):
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
