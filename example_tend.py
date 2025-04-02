#!/usr/bin/env python3
"""
Example program for using the Robotiq Hand-E C10 gripper in a machine loading task
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger, SetBool

# Import custom messages and services
from robotiq_hand_e_driver.msg import GripperCommand
from robotiq_hand_e_driver.srv import GripperActivation

# Robot motion control interface
from robot_motion_interface import RobotMotionClient

# Machine interface
from machine_interface import LatheInterface


class PartLoadingNode(Node):
    """Node for controlling the robotic part loading task."""

    def __init__(self):
        super().__init__('part_loading_node')

        # Create clients for gripper services
        self.gripper_activate_client = self.create_client(
            GripperActivation, 'gripper/activate')
        self.gripper_open_client = self.create_client(
            SetBool, 'gripper/open')
        self.gripper_close_client = self.create_client(
            SetBool, 'gripper/close')

        # Create publisher for gripper commands
        self.gripper_command_pub = self.create_publisher(
            GripperCommand, 'gripper/command', 10)

        # Robot motion client
        self.robot_client = RobotMotionClient()

        # Lathe machine interface
        self.lathe = LatheInterface()

        # Initialize the system
        self.initialize()

    def initialize(self):
        """Initialize the system."""
        self.get_logger().info("Initializing system")

        # Ensure services are available
        self.gripper_activate_client.wait_for_service()
        self.gripper_open_client.wait_for_service()
        self.gripper_close_client.wait_for_service()

        # Activate the gripper
        activate_request = GripperActivation.Request()
        activate_request.activate = True
        self.gripper_activate_client.call_async(activate_request)
        self.get_logger().info("Gripper activated")

        # Move robot to home position
        self.robot_client.move_to_position("home_position")

    def run_task(self):
        """Run the complete part loading task."""
        self.get_logger().info("Starting part loading task")

        # Pick pre-machined part
        self.pick_pre_machined_part()

        # Load part into lathe
        self.load_part_into_lathe()

        # Wait for machining to complete
        self.wait_for_machining()

        # Pick post-machined part
        self.pick_post_machined_part()

        # Place post-machined part
        self.place_post_machined_part()

        self.get_logger().info("Task completed successfully")

    def pick_pre_machined_part(self):
        """Pick up the pre-machined part."""
        self.get_logger().info("Picking pre-machined part")

        # Check that machine program is finished
        if not self.lathe.is_program_finished():
            self.get_logger().warn("Machine program not finished, waiting...")
            self.lathe.wait_for_program_finish()

        # Move robot to home position
        self.robot_client.move_to_position("home_position")

        # Move to approach position
        approach_pos = self.get_part_approach_position()
        self.robot_client.move_to_position(approach_pos)

        # Open gripper fully with medium speed, high force
        self.open_gripper(speed=150, force=250)

        # Move to pick position
        pick_pos = self.get_part_pick_position()
        self.robot_client.move_to_position(pick_pos)

        # Close gripper to grasp part with medium speed, high force
        self.close_gripper(speed=150, force=250)

        # Check if object was grasped
        if not self.is_object_grasped():
            self.get_logger().error("Failed to grasp part")
            # Handle error

        # Move back to approach position
        self.robot_client.move_to_position(approach_pos)

    def load_part_into_lathe(self):
        """Load the part into the lathe."""
        self.get_logger().info("Loading part into lathe")

        # Open lathe door
        self.lathe.open_door()

        # Open collet
        self.lathe.open_collet()

        # Plan motion from current position to approach collet position
        path = self.plan_motion(
            self.robot_client.get_current_position(),
            "approach_collet_position"
        )

        # Execute the planned path
        self.robot_client.execute_path(path)

        # Move to place position
        self.robot_client.move_to_position("place_part_in_collet_position")

        # Close collet
        self.lathe.close_collet()

        # Open gripper with medium speed to release part
        self.open_gripper(speed=150, force=200)

        # Move back to approach position
        self.robot_client.move_to_position("approach_collet_position")

        # Move to home position
        self.robot_client.move_to_position("home_position")

        # Close lathe door
        self.lathe.close_door()

    def wait_for_machining(self):
        """Wait for the machining process to complete."""
        self.get_logger().info("Starting machining program")

        # Start the machine program
        self.lathe.start_program()

        # Monitor the state of the machine
        while not self.lathe.is_program_finished():
            self.get_logger().info("Machining in progress...")
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Machining completed")

    def pick_post_machined_part(self):
        """Pick up the post-machined part."""
        self.get_logger().info("Picking post-machined part")

        # Open lathe door
        self.lathe.open_door()

        # Plan motion from home to approach collet
        path = self.plan_motion("home_position", "approach_collet_position")

        # Execute the planned path
        self.robot_client.execute_path(path)

        # Capture image for grasp planning
        image = self.robot_client.capture_image()

        # Automated grasp planning
        grasp_pos = self.automated_grasp_planning(image)

        # Move to grasp position
        self.robot_client.move_to_position(grasp_pos)

        # Close gripper with medium speed and force to grasp part
        self.close_gripper(speed=150, force=200)

        # Check if object was grasped
        if not self.is_object_grasped():
            self.get_logger().error("Failed to grasp post-machined part")
            # Handle error

        # Move back to approach position
        self.robot_client.move_to_position("approach_collet_position")

        # Move to home position
        self.robot_client.move_to_position("home_position")

    def place_post_machined_part(self):
        """Place the post-machined part."""
        self.get_logger().info("Placing post-machined part")

        # Move to approach position
        approach_pos = self.get_output_approach_position()
        self.robot_client.move_to_position(approach_pos)

        # Move to place position
        place_pos = self.get_output_place_position()
        self.robot_client.move_to_position(place_pos)

        # Open gripper to release part
        self.open_gripper(speed=100, force=150)

    # Gripper control methods
    def open_gripper(self, speed=255, force=255):
        """Open the gripper with specified speed and force."""
        request = SetBool.Request()
        request.data = True  # True to wait for completion

        # Send command with detailed parameters
        cmd = GripperCommand()
        cmd.position = 40.0  # 40mm is fully open
        cmd.speed = speed
        cmd.force = force
        self.gripper_command_pub.publish(cmd)

        # Call service to wait for completion
        future = self.gripper_open_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("Gripper opened")

    def close_gripper(self, speed=255, force=255):
        """Close the gripper with specified speed and force."""
        request = SetBool.Request()
        request.data = True  # True to wait for completion

        # Send command with detailed parameters
        cmd = GripperCommand()
        cmd.position = -1.0  # Negative position means close
        cmd.speed = speed
        cmd.force = force
        self.gripper_command_pub.publish(cmd)

        # Call service to wait for completion
        future = self.gripper_close_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.get_logger().info("Gripper closed")

    def is_object_grasped(self):
        """Check if an object is currently grasped."""
        # Implementation would check the gripper status topic
        # to see if an object was detected during closing
        return True  # Simplified for this example

    # Position and planning methods
    def get_part_approach_position(self):
        """Get the approach position for the part."""
        # This would normally access a database or hard-coded position
        return "approach_part_position"

    def get_part_pick_position(self):
        """Get the pick position for the part."""
        # This would normally access a database or hard-coded position
        return "pick_part_position"

    def get_output_approach_position(self):
        """Get the approach position for placing the output part."""
        # This would normally access a database or hard-coded position
        return "output_approach_position"

    def get_output_place_position(self):
        """Get the place position for the output part."""
        # This would normally access a database or hard-coded position
        return "output_place_position"

    def plan_motion(self, pos1, pos2):
        """Plan a motion path between two positions."""
        # This would normally compute an optimized path
        return [pos1, pos2]  # Simplified for this example

    def automated_grasp_planning(self, image):
        """Perform automated grasp planning based on an image."""
        # This would normally analyze the image and compute a grasp pose
        return "computed_grasp_position"  # Simplified for this example


def main(args=None):
    rclpy.init(args=args)

    node = PartLoadingNode()

    try:
        node.run_task()
    except Exception as e:
        node.get_logger().error(f"Task failed with error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()