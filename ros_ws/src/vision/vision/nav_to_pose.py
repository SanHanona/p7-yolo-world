from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
"""

class Commander(Node):
    def __init__(self):
        super().__init__('commander')

        # Subscribtion to the action command topic for, well... commands
        self.depth_subscription = self.create_subscription(
            String,
            '/command/action',
            self.action_callback,
            10)
        
        # BasicNavigator comes from the nav2 commander package and is what we use to create and cancel navigation goals
        self.navigator = BasicNavigator()

        # Initialize the timer function that handle the status of ongoing tasks
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Used for tracking whether there is an ongoing navigation task
        self.nav_flag = False

        # Variable for swithcing between the goals
        self.goalnr = 0

        self.map = "Hospital"

        self.initial_pose = PoseStamped()
        self.goal1_pose = PoseStamped()
        self.goal2_pose = PoseStamped()

        if self.map == "Warehouse":
            # Set our initial pose
            self.initial_pose.header.frame_id = 'map'
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.initial_pose.pose.position.x = -6.0
            self.initial_pose.pose.position.y = 0.0
            self.initial_pose.pose.orientation.z = 3.1415
            self.initial_pose.pose.orientation.w = 1.0
            self.navigator.setInitialPose(self.initial_pose)

            # Wait for navigation to fully activate
            self.navigator.waitUntilNav2Active()

            # Set our first goal pose
            self.goal1_pose.header.frame_id = 'map'
            self.goal1_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.goal1_pose.pose.position.x = 0.0
            self.goal1_pose.pose.position.y = 15.0
            self.goal1_pose.pose.orientation.w = 1.0
            self.goal1_pose.pose.orientation.z = 0.0

            # Set our second goal pose
            self.goal2_pose.header.frame_id = 'map'
            self.goal2_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.goal2_pose.pose.position.x = 0.0
            self.goal2_pose.pose.position.y = -9.0
            self.goal2_pose.pose.orientation.w = 1.0
            self.goal2_pose.pose.orientation.z = 0.0
        
        if self.map == "Hospital":
            # Set our initial pose
            self.initial_pose.header.frame_id = 'map'
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.initial_pose.pose.position.x = 8.0
            self.initial_pose.pose.position.y = 6.0
            self.initial_pose.pose.orientation.z = 3.1415
            self.initial_pose.pose.orientation.w = 1.0
            self.navigator.setInitialPose(self.initial_pose)

            # Wait for navigation to fully activate
            self.navigator.waitUntilNav2Active()

            # Set our first goal pose
            self.goal1_pose.header.frame_id = 'map'
            self.goal1_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.goal1_pose.pose.position.x = 5.0
            self.goal1_pose.pose.position.y = 10.5
            self.goal1_pose.pose.orientation.w = 1.0
            self.goal1_pose.pose.orientation.z = 1.57

            # Set our second goal pose
            self.goal2_pose.header.frame_id = 'map'
            self.goal2_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.goal2_pose.pose.position.x = -25.0
            self.goal2_pose.pose.position.y = 10.5
            self.goal2_pose.pose.orientation.w = 1.0
            self.goal2_pose.pose.orientation.z = -1.57
        
        self.get_logger().info("Commander node initialized, using map: " + self.map)

    # Runs whenever a commnad is published
    def action_callback(self, msg):
        command = msg.data
        self.get_logger().info("Recived command " + command)

        # If the command is 'stop', cancel the active task
        if command[:4] == 'stop':
            self.nav_flag = False
            self.navigator.cancelTask()

        # If the command is 'wait', cancel the active task, wait for the set time and the reset the goal pose
        if command[:4] == 'wait':
            self.nav_flag = False
            self.navigator.cancelTask()
            wait_time = int(command[4:])
            # self.get_logger().info('Feeling sleepy')
            time.sleep(wait_time)
            # self.get_logger().info('Waking up!')
            self.set_goalpose()

        # If the command is 'pass', reset the goal pose
        if command[:4] == 'pass':
            self.nav_flag = True
            self.set_goalpose()

    # Handles the setting of goal poses, swithcing between the two poses once they are reached
    def set_goalpose(self):
        if self.goalnr == 0:
            path = self.navigator.getPath(self.initial_pose, self.goal1_pose)

            self.navigator.goToPose(self.goal1_pose)
        elif self.goalnr == 1:
            path = self.navigator.getPath(self.goal2_pose, self.goal1_pose)

            self.navigator.goToPose(self.goal1_pose)
        elif self.goalnr == 2:
            path = self.navigator.getPath(self.goal1_pose, self.goal2_pose)

            self.navigator.goToPose(self.goal2_pose)
        
    # Handles keeping the status of the running task, and detecting once it has finished
    def timer_callback(self):
        done = False
        # If navigation is in progress, get and print the ETA from the feedback
        if self.nav_flag:
            feedback = self.navigator.getFeedback()
            if feedback != None:
                self.get_logger().info(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )
                # Automatic cancelaiton after 10 minutes
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
                 
                # If the task is complete, this variable becomes true 
                done = self.navigator.isTaskComplete()


        # Runs whenerver a task is finished
        if done:
            
            result = self.navigator.getResult()

            # If the task was successful, update the goal number
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                self.nav_flag = False
                if self.goalnr == 0:
                    self.goalnr = 2
                elif self.goalnr == 2:
                    self.goalnr = 1
                elif self.goalnr == 1:
                    self.goalnr = 2

            # If the task failed or was canceled simply print this to the terminal without changing goal number
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
                self.nav_flag = False
            elif result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')
                self.nav_flag = False
            else:
                self.get_logger().info('Goal has an invalid return status!')


def main(args=None):
    rclpy.init(args=args)
    action_commander = Commander()
    rclpy.spin(action_commander)
    action_commander.destroy_node()
    rclpy.shutdown()

    if __name__ == '__main__':
        main()