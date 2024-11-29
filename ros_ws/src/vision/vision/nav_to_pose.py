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
        self.get_logger().info("Commander node initialized.")

        self.depth_subscription = self.create_subscription(
            String,
            '/command/action',
            self.action_callback,
            10)
        
        self.navigator = BasicNavigator()

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.nav_flag = False

        self.goalnr = 0

        # Set our demo's initial pose
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = -6.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.z = 3.1415
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        # Go to our demos first goal pose
        self.goal1_pose = PoseStamped()
        self.goal1_pose.header.frame_id = 'map'
        self.goal1_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal1_pose.pose.position.x = 0.0
        self.goal1_pose.pose.position.y = 15.0
        self.goal1_pose.pose.orientation.w = 1.0
        self.goal1_pose.pose.orientation.z = 0.0

        self.goal2_pose = PoseStamped()
        self.goal2_pose.header.frame_id = 'map'
        self.goal2_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal2_pose.pose.position.x = 0.0
        self.goal2_pose.pose.position.y = -9.0
        self.goal2_pose.pose.orientation.w = 1.0
        self.goal2_pose.pose.orientation.z = 0.0


    def action_callback(self, msg):
        command = msg.data
        self.get_logger().info("Recived command " + command)

        if command[:4] == 'stop':
            self.nav_flag = False
            self.navigator.cancelTask()

        if command[:4] == 'wait':
            self.nav_flag = False
            self.navigator.cancelTask()
            wait_time = int(command[4:])
            # self.get_logger().info('Feeling sleepy')
            time.sleep(wait_time)
            # self.get_logger().info('Waking up!')
            self.set_goalpose()

        if command[:4] == 'pass':
            self.nav_flag = True
            self.set_goalpose()


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
        

    def timer_callback(self):
        done = False
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
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()
                done = self.navigator.isTaskComplete()

        if done:
            self.nav_flag = False
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                if self.goalnr == 0:
                    self.goalnr = 2
                elif self.goalnr == 2:
                    self.goalnr = 1
                elif self.goalnr == 1:
                    self.goalnr = 2

            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')
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


# def main():
#     rclpy.init()

#     navigator = BasicNavigator()

#     # Set our demo's initial pose
#     initial_pose = PoseStamped()
#     initial_pose.header.frame_id = 'map'
#     initial_pose.header.stamp = navigator.get_clock().now().to_msg()
#     initial_pose.pose.position.x = -6.0
#     initial_pose.pose.position.y = 0.0
#     initial_pose.pose.orientation.z = 3.1415
#     initial_pose.pose.orientation.w = 1.0
#     navigator.setInitialPose(initial_pose)

#     # Activate navigation, if not autostarted. This should be called after setInitialPose()
#     # or this will initialize at the origin of the map and update the costmap with bogus readings.
#     # If autostart, you should `waitUntilNav2Active()` instead.
#     # navigator.lifecycleStartup()

#     # Wait for navigation to fully activate, since autostarting nav2
#     navigator.waitUntilNav2Active()

#     # If desired, you can change or load the map as well
#     # navigator.changeMap('/path/to/map.yaml')

#     # You may use the navigator to clear or obtain costmaps
#     # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
#     # global_costmap = navigator.getGlobalCostmap()
#     # local_costmap = navigator.getLocalCostmap()

#     # Go to our demos first goal pose
#     goal1_pose = PoseStamped()
#     goal1_pose.header.frame_id = 'map'
#     goal1_pose.header.stamp = navigator.get_clock().now().to_msg()
#     goal1_pose.pose.position.x = 0.0
#     goal1_pose.pose.position.y = 15.0
#     goal1_pose.pose.orientation.w = 1.0
#     goal1_pose.pose.orientation.z = 0.0

#     goal2_pose = PoseStamped()
#     goal2_pose.header.frame_id = 'map'
#     goal2_pose.header.stamp = navigator.get_clock().now().to_msg()
#     goal2_pose.pose.position.x = 0.0
#     goal2_pose.pose.position.y = -9.0
#     goal2_pose.pose.orientation.w = 1.0
#     goal2_pose.pose.orientation.z = 0.0

#     # sanity check a valid path exists
#     path = navigator.getPath(initial_pose, goal1_pose)

#     navigator.goToPose(goal1_pose)

#     i = 0
#     while not navigator.isTaskComplete():
#         ################################################
#         #
#         # Implement some code here for your application!
#         #
#         ################################################

#         # Do something with the feedback
#         i = i + 1
#         feedback = navigator.getFeedback()
#         if feedback and i % 5 == 0:
#             print(
#                 'Estimated time of arrival: '
#                 + '{0:.0f}'.format(
#                     Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
#                     / 1e9
#                 )
#                 + ' seconds.'
#             )

#             # Some navigation timeout to demo cancellation
#             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
#                 navigator.cancelTask()

#             # Some navigation request change to demo preemption
#             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
#                 goal1_pose.pose.position.x = 0.0
#                 goal1_pose.pose.position.y = 0.0
#                 navigator.goToPose(goal1_pose)

#     # Do something depending on the return code
#     result = navigator.getResult()
#     if result == TaskResult.SUCCEEDED:
#         print('Goal succeeded!')
#     elif result == TaskResult.CANCELED:
#         print('Goal was canceled!')
#     elif result == TaskResult.FAILED:
#         print('Goal failed!')
#     else:
#         print('Goal has an invalid return status!')

#     navigator.lifecycleShutdown()

#     exit(0)


# if __name__ == '__main__':
#     main()
