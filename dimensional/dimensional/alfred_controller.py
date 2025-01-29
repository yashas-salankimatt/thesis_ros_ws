import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class AlfredController(Node):
    def __init__(self):
        super().__init__('alfred_controller')

        # Subscriptions
        self.create_subscription(Twist, '/alfred_base_center_tf_twist', self.base_twist_callback, 10)
        self.create_subscription(Twist, '/movexy_goal', self.goal_callback, 10)

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/alfred_diff_cont/cmd_vel_unstamped', 10)

        # Variables to store the latest messages
        self.base_twist = None
        self.goal_twist = None

        # Timer to process control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def base_twist_callback(self, msg):
        self.base_twist = msg

    def goal_callback(self, msg):
        self.goal_twist = msg

    def calculate_angle_to_goal(self):
        """
        Calculate the angle the robot needs to rotate to face the goal xy position.
        """
        dx = self.goal_twist.linear.x - self.base_twist.linear.x
        dy = self.goal_twist.linear.y - self.base_twist.linear.y
        target_angle = math.degrees(math.atan2(dy, dx))
        current_angle = self.base_twist.angular.z
        if current_angle > -90:
            current_angle = 90 + current_angle
        else:
            current_angle = -(abs(current_angle) - 90)
        if target_angle > 90 and current_angle < -90:
            current_angle = 360 + current_angle
        if target_angle < -90 and current_angle > 90:
            target_angle = 360 + target_angle
        # print(f"Target angle: {target_angle}")
        # print(f"Current angle: {current_angle}")

        # Normalize angles to [-pi, pi]
        angular_difference = target_angle - current_angle
        # angular_difference = math.atan2(math.sin(angular_difference), math.cos(angular_difference))
        # print(f"Angular difference: {angular_difference}")

        return angular_difference

    def control_loop(self):
        if self.base_twist is None or self.goal_twist is None:
            return


        # Calculate distance and angular difference
        linear_distance = math.sqrt(
            (self.goal_twist.linear.x - self.base_twist.linear.x) ** 2 +
            (self.goal_twist.linear.y - self.base_twist.linear.y) ** 2
        )
        angular_difference = self.calculate_angle_to_goal()
        # raw_angular_difference = self.goal_twist.angular.z - self.base_twist.angular.z
        current_angle = self.base_twist.angular.z
        if current_angle > -90:
            current_angle = 90 + current_angle
        else:
            current_angle = -(abs(current_angle) - 90)

        
        # the goal angle is defined as positive counterclockwise or negative clockwise from the x axis
        current_goal_angle = self.goal_twist.angular.z
        raw_angular_difference = current_goal_angle - current_angle

        twist_cmd = Twist()

        # print(f"Linear distance: {linear_distance}")
        # print(f"Raw angular difference: {raw_angular_difference}")
        if linear_distance > 0.05:
            # Rotate towards the goal
            if abs(angular_difference) > 0.5:
                twist_cmd.angular.z = max(-1.0, min(1.0, angular_difference / 10))
            else:
                # Move straight towards the goal
                twist_cmd.linear.x = 0.5
                # print(f"Linear distance: {linear_distance}")
        else:
            # print(f"Target angle: {current_goal_angle}")
            # print(f"Current angle: {current_angle}")
            # print(f"Raw angular difference: {raw_angular_difference}")
            # Stop and align with the angular goal
            if abs(raw_angular_difference) > 0.5:
                twist_cmd.angular.z = max(-1.0, min(1.0, raw_angular_difference / 10))
            else:
                self.goal_twist = None
                # print("Movexy goal reached")

        self.cmd_vel_publisher.publish(twist_cmd)


def main(args=None):
    rclpy.init(args=args)
    controller = AlfredController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()