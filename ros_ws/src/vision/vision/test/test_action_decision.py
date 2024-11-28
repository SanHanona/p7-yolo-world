import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time


class ActionDecisionTester(Node):
    def __init__(self):
        super().__init__('action_decision_tester')
        self.publisher_attention = self.create_publisher(Bool, '/attention', 10)
        self.publisher_language = self.create_publisher(String, '/command/language', 10)
        self.command_window = 0.5  # Time interval between commands in seconds

    def publish_attention(self, state, duration=1.0):
        """Publish attention state for a specified duration."""
        self.get_logger().info(f"Publishing attention: {state}")
        msg = Bool(data=state)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_attention.publish(msg)
            time.sleep(self.command_window)

    def publish_language(self, command, repeat=1):
        """Publish language commands with optional repetitions."""
        self.get_logger().info(f"Publishing language command: {command}")
        msg = String(data=command)
        for _ in range(repeat):
            self.publisher_language.publish(msg)
            time.sleep(self.command_window)

    def test_stop_command(self):
        """Test stop command."""
        self.get_logger().info("Testing: stop command")
        self.publish_language('stop')

    def test_wait_command(self):
        """Test wait command."""
        self.get_logger().info("Testing: wait command")
        self.publish_language('wait')

    def test_pass_command(self):
        """Test pass command logic."""
        self.get_logger().info("Testing: pass command")
        self.publish_language('pass', repeat=3)

    def test_attention_true(self):
        """Test attention True."""
        self.get_logger().info("Testing: attention True")
        self.publish_attention(True)

    def test_attention_false(self):
        """Test attention False."""
        self.get_logger().info("Testing: attention False")
        self.publish_attention(False)

    def test_attention_loss(self):
        """Test attention loss."""
        self.get_logger().info("Testing: attention loss")
        self.publish_attention(True, duration=1.0)  # Publish attention briefly
        time.sleep(3)  # Simulate loss of attention

    def run_all_tests(self):
        """Run all test cases in sequence."""
        self.test_stop_command()
        time.sleep(1)
        self.test_wait_command()
        time.sleep(1)
        self.test_pass_command()
        time.sleep(1)
        self.test_attention_true()
        time.sleep(1)
        self.test_attention_false()
        time.sleep(1)
        self.test_attention_loss()
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    tester = ActionDecisionTester()
    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted.")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
