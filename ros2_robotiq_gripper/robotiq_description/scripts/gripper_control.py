#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
from time import sleep

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control_node')
        # Connect to the gripper action server on '/robotiq_gripper_controller/gripper_cmd'
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self.get_logger().info('Gripper control node started')

    def send_gripper_command(self, position: float):
        # Wait for the action server to be available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server to be available...')
        
        # Prepare the goal message with the gripper's position command
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position  # 0.0 for open, 1.0 for closed
        goal_msg.command.max_effort = 50.0  # Adjust based on your gripper's need

        # Send the goal to the action server
        self.get_logger().info(f'Sending gripper command: {position}')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    gripper_control = GripperControl()

    try:
        # Close the gripper
        gripper_control.send_gripper_command(0.0)  # 0.0 for open, 1.0 for closed
        sleep(3)  # Wait for 3 seconds
        # Open the gripper
        gripper_control.send_gripper_command(1.0)
    except KeyboardInterrupt:
        pass

    gripper_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
