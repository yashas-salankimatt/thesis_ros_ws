import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup


class GripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_action_client')
        
        # Create action client for the gripper action
        self.action_client = ActionClient(self, GripperCommand, '/xarm_gripper/gripper_action')
        
        # Create a callback group to allow the subscription and action to be handled in parallel
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to the /gripper_position topic
        self.subscription = self.create_subscription(
            Float32,
            '/gripper_position',
            self.gripper_position_callback,
            10,
            callback_group=self.callback_group
        )

    def gripper_position_callback(self, msg):
        gripper_position = msg.data
        self.get_logger().info(f'Received gripper position: {gripper_position}')

        # Send the gripper action request with the received position
        self.send_gripper_goal(gripper_position)

    def send_gripper_goal(self, position):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 10.0  # You can adjust max effort if needed

        self.get_logger().info(f'Sending goal to gripper: position = {position}')

        # Wait for the action server to be available
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        # Send the goal
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Gripper action goal rejected.')
            return

        self.get_logger().info('Gripper action goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Gripper action result: {result}')


def main(args=None):
    rclpy.init(args=args)
    
    gripper_action_client = GripperActionClient()

    try:
        rclpy.spin(gripper_action_client)
    except KeyboardInterrupt:
        gripper_action_client.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        gripper_action_client.get_logger().error(f'Exception in node: {e}')
    finally:
        gripper_action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
