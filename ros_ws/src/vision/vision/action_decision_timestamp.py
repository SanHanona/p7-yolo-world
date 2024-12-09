import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
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

        self.publish_pass_state = self.create_publisher(Bool, '/pass_state', 10)

        # Parameter declaration
        self.cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service aquired!')
        self.req = SetParameters.Request()


        # State Variables
        self.last_attention_command = None
        self.last_attention_timestamp = None

        self.last_language_command = None
        self.last_language_timestamp = None

        self.last_gesture_command = None
        self.last_gesture_timestamp = None

        self.pass_state = False
        self.pass_command_count = 0  

        self.safety_distance = None
        self.last_distance_timestamp = None

        self.command_window = 10.0  # Command validity in seconds
        self.wait_time_after_attention_lost = 10.0  # Wait time before deactivating pass_state

        self.timer = self.create_timer(1, self.timer_callback)

    def attention_callback(self, msg):
        self.last_attention_command = msg.data  
        self.last_attention_timestamp = self.get_clock().now()
        # self.get_logger().info(f"Attention state updated: {self.last_attention_command}")

    def language_callback(self, msg):
        self.last_language_command = msg.data
        self.last_language_timestamp = self.get_clock().now()
        self.get_logger().info(f"Language received: {self.last_language_command}")

    def gesture_callback(self, msg):
        self.last_gesture_command = msg.data
        if self.last_gesture_command == "Thumbs up":
            self.last_gesture_command = "pass"
        self.last_gesture_timestamp = self.get_clock().now()
        self.get_logger().info(f"Gesture received: {self.last_gesture_command}")


    def distance_callback(self, msg):
        self.safety_distance = msg.data
        self.last_distance_timestamp = self.get_clock().now()
        # self.get_logger().info(f"Distance received: {self.safety_distance}")

    def is_distance_within_range(self):
        """Check if the distance is within the stopping range."""
        if self.safety_distance is None or self.last_distance_timestamp is None:
            return False
        # self.get_logger().info(f"Distance check: {self.safety_distance <= 5}")
        return self.safety_distance <= 5 # change if needed 


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
        if (self.pass_state is True) and (attention is None):  
            time_since_last_attention = (
                (current_time - self.last_attention_timestamp).nanoseconds / 1e9
                if self.last_attention_timestamp else float('inf')
            )
            self.get_logger().info(f"pass state {self.pass_state} - attention is {attention}")
            
            if time_since_last_attention > self.wait_time_after_attention_lost:
                if self.pass_state:
                    self.get_logger().info("Option 1")
                    self.set_max_speed(1.8, 1)
                self.pass_state = False
                # param = Parameter('/controller_server FollowPath.max_vel_x', Parameter.Type.DOUBLE, 1.8)
                # self.set_parameters([param])
                self.publish_pass_state.publish(Bool(data=False))
                self.safety_distance = 1000
                self.get_logger().info("Pass state deactivated due to attention loss.")
        
        if language == "pass" or gesture == "pass":
            
            self.pass_command_count += 1
            if self.pass_command_count >= 3:
                if not self.pass_state:
                    self.get_logger().info("Option 2")
                    self.set_max_speed(0.3, 0.25)
                self.pass_state = True
                # param = Parameter('/controller_server FollowPath.max_vel_x', Parameter.Type.DOUBLE, 0.6)
                # self.set_parameters([param])
                self.publish_pass_state.publish(Bool(data=True))
                self.get_logger().info("Pass state activated.")
        else:
            self.pass_command_count = 0

    def decide_action(self, language, gesture, attention):
        """Decide on the action based on the hierarchy."""

        is_close = self.is_distance_within_range()
        if is_close and not self.pass_state:
            self.stop_action()

        if attention or self.pass_state:
            if "stop" in {gesture, language}:
                self.stop_action()
            elif "wait" in {gesture, language}:
                self.wait_action()
            elif "pass" in {gesture, language}: #or attention in [True, False]:  # Handles attention True/False explicitly
                self.pass_action()
            else:
                self.get_logger().info("No valid commands or conditions met.")
        elif attention == False:
            if is_close:
                # self.stop_action()
                self.get_logger().info("No Attention.")
                self.get_attention()
            else:
                self.get_logger().info(f"Person detected at {self.safety_distance} m")
        # else: 
        #     self.get_logger().info("action handler passive")

    def get_attention(self): 
        self.get_logger().info("*LOUD BEEP*")
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

    def set_max_speed(self, speed, theta):
        param = Parameter(
            name='FollowPath.max_vel_x', 
            value=ParameterValue(
                type=3,  # Type 3 corresponds to float64
                double_value=speed
            )
        )
        param2 = Parameter(
            name='FollowPath.max_vel_theta', 
            value=ParameterValue(
                type=3,  # Type 3 corresponds to float64
                double_value=theta
            )
        )

        # Create the request and set the parameters
        request = SetParameters.Request()
        request.parameters = [param, param2]

        # Send the request and wait for the response
        future = self.cli.call_async(request)
        future.add_done_callback(self.set_spd_callback)

    def set_spd_callback(self, msg):
        self.get_logger().info("Speed changed")


def main(args=None):
    rclpy.init(args=args)
    action_decision = ActionDecision()
    rclpy.spin(action_decision)
    action_decision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
