#!/usr/bin/env python3
"""
Example of moving to a pose goal.
- Moves the robot 20 cm forward along the x-axis and then 20 cm backward.
"""

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import panda


def move_robot_to_position(moveit2, position, quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold):
    """
    Move the robot to a specified position with given parameters.
    """
    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        cartesian=cartesian,
        cartesian_max_step=cartesian_max_step,
        cartesian_fraction_threshold=cartesian_fraction_threshold,
    )


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.25, 0.0, 0.5])  # Starting position
    node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("synchronous", True)
    node.declare_parameter("cancel_after_secs", 0.0)
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")
    node.declare_parameter("cartesian", False)
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda.joint_names(),
        base_link_name=panda.base_link_name(),
        end_effector_name=panda.end_effector_name(),
        group_name=panda.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.planner_id = node.get_parameter("planner_id").get_parameter_value().string_value

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    cartesian_fraction_threshold = node.get_parameter("cartesian_fraction_threshold").get_parameter_value().double_value
    cartesian_jump_threshold = node.get_parameter("cartesian_jump_threshold").get_parameter_value().double_value
    cartesian_avoid_collisions = node.get_parameter("cartesian_avoid_collisions").get_parameter_value().bool_value

    # Set parameters for cartesian planning
    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # Initial position
    initial_position = list(position)
    move_robot_to_position(moveit2, initial_position, quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)

    # Move 20 cm forward (x+ direction)
    forward_position = [initial_position[0] + 0.2, initial_position[1], initial_position[2]]
    node.get_logger().info(f"Moving forward to position: {forward_position}")
    move_robot_to_position(moveit2, forward_position, quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)

    # Move 20 cm back (x- direction)
    backward_position = [forward_position[0] - 0.2, forward_position[1], forward_position[2]]
    node.get_logger().info(f"Moving back to position: {backward_position}")
    move_robot_to_position(moveit2, backward_position, quat_xyzw, cartesian, cartesian_max_step, cartesian_fraction_threshold)

    # Wait for the motion to complete
    if synchronous:
        moveit2.wait_until_executed()
    else:
        # Wait for the request to get accepted (i.e., for execution to start)
        print("Current State: " + str(moveit2.query_state()))
        rate = node.create_rate(10)
        while moveit2.query_state() != MoveIt2State.EXECUTING:
            rate.sleep()

        future = moveit2.get_execution_future()

        # Cancel the goal
        if cancel_after_secs > 0.0:
            sleep_time = node.create_rate(cancel_after_secs)
            sleep_time.sleep()
            print("Cancelling goal")
            moveit2.cancel_execution()

        while not future.done():
            rate.sleep()

        # Print the result
        print("Result status: " + str(future.result().status))
        print("Result error code: " + str(future.result().result.error_code))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
