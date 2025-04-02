#!/usr/bin/env python3
"""
ROS 2 Node for the Robotiq Hand-E C10 Gripper.
Provides topics and services to control the gripper.
"""

import time
import threading
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float32, UInt8
from std_srvs.srv import Trigger, SetBool

# Import custom messages
from robotiq_hand_e_driver.msg import GripperCommand, GripperStatus
from robotiq_hand_e_driver.srv import GripperActivation, GripperEmergencyRelease

# Import the driver
from robotiq_hand_e_driver.robotiq_hand_e_driver import RobotiqHandEDriver


class HandENode(Node):
    """ROS 2 Node for controlling the Robotiq Hand-E C10 Gripper."""

    def __init__(self):
        """Initialize the ROS 2 node."""
        super().__init__('robotiq_hand_e_node')

        # Create callback group for services
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('ip_address', '192.168.1.2')
        self.declare_parameter('tcp_port', 502)
        self.declare_parameter('slave_id', 9)
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('auto_connect', True)
        self.declare_parameter('auto_activate', False)

        # Get parameters
        ip_address = self.get_parameter('ip_address').value
        tcp_port = self.get_parameter('tcp_port').value
        slave_id = self.get_parameter('slave_id').value
        update_rate = self.get_parameter('update_rate').value
        auto_connect = self.get_parameter('auto_connect').value
        auto_activate = self.get_parameter('auto_activate').value

        self.get_logger().info(f"Initializing Robotiq Hand-E C10 driver with IP: {ip_address}, Port: {tcp_port}")

        # Initialize driver
        self.driver = RobotiqHandEDriver(
            ip_address=ip_address,
            port=tcp_port,
            slave_id=slave_id,
            debug=False,
            logger=self.get_logger()
        )

        # Initialize state
        self.connected = False
        self.activated = False
        self.last_status = None
        self.lock = threading.Lock()

        # Create publishers
        self.status_pub = self.create_publisher(GripperStatus, 'gripper/status', 10)
        self.position_pub = self.create_publisher(Float32, 'gripper/position', 10)
        self.object_detected_pub = self.create_publisher(Bool, 'gripper/object_detected', 10)

        # Create subscribers
        self.command_sub = self.create_subscription(
            GripperCommand,
            'gripper/command',
            self.command_callback,
            10
        )
        self.position_sub = self.create_subscription(
            Float32,
            'gripper/position_command',
            self.position_callback,
            10
        )

        # Create services
        self.activate_srv = self.create_service(
            GripperActivation,
            'gripper/activate',
            self.activate_callback,
            callback_group=self.callback_group
        )

        self.reset_srv = self.create_service(
            Trigger,
            'gripper/reset',
            self.reset_callback,
            callback_group=self.callback_group
        )

        self.emergency_release_srv = self.create_service(
            GripperEmergencyRelease,
            'gripper/emergency_release',
            self.emergency_release_callback,
            callback_group=self.callback_group
        )

        self.open_srv = self.create_service(
            SetBool,
            'gripper/open',
            self.open_callback,
            callback_group=self.callback_group
        )

        self.close_srv = self.create_service(
            SetBool,
            'gripper/close',
            self.close_callback,
            callback_group=self.callback_group
        )

        # Create timer for status updates
        self.update_timer = self.create_timer(
            1.0 / update_rate,
            self.update_status,
            callback_group=self.callback_group
        )

        # Auto-connect and activate if requested
        if auto_connect:
            self.connect()
            if auto_activate and self.connected:
                success = self.activate()
                self.get_logger().info(f"Auto-activation {'succeeded' if success else 'failed'}")

        self.get_logger().info("Robotiq Hand-E C10 node initialized")

    def connect(self) -> bool:
        """
        Connect to the gripper.

        Returns:
            True if connection succeeded, False otherwise
        """
        with self.lock:
            self.connected = self.driver.connect()
            if self.connected:
                self.get_logger().info("Connected to gripper")
            else:
                self.get_logger().error("Failed to connect to gripper")
            return self.connected

    def disconnect(self) -> None:
        """Disconnect from the gripper."""
        with self.lock:
            self.driver.disconnect()
            self.connected = False
            self.activated = False
            self.get_logger().info("Disconnected from gripper")

    def activate(self) -> bool:
        """
        Activate the gripper.

        Returns:
            True if activation succeeded, False otherwise
        """
        with self.lock:
            if not self.connected and not self.connect():
                self.get_logger().error("Cannot activate, not connected")
                return False

            self.get_logger().info("Activating gripper...")
            success = self.driver.activate_and_wait()
            self.activated = success

            if success:
                self.get_logger().info("Gripper activated successfully")
            else:
                self.get_logger().error("Failed to activate gripper")

            return success

    def update_status(self) -> None:
        """Update and publish gripper status."""
        if not self.connected:
            return

        with self.lock:
            status_dict = self.driver.get_status()

        if not status_dict:
            self.get_logger().warn("Failed to get gripper status")
            return

        # Store last status
        self.last_status = status_dict
        self.activated = status_dict['activated']

        # Create and publish status message
        status_msg = GripperStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.activated = status_dict['activated']
        status_msg.in_motion = status_dict['object_status'] == 'MOVING'
        status_msg.object_detected = status_dict['object_status'] in ['OBJECT_DETECTED_OPENING',
                                                                      'OBJECT_DETECTED_CLOSING']
        status_msg.position = status_dict['position']
        status_msg.position_mm = status_dict['position_mm']
        status_msg.current = status_dict['current']
        status_msg.fault_status = status_dict['fault_status']
        status_msg.status = status_dict['gripper_status']

        self.status_pub.publish(status_msg)

        # Publish position
        pos_msg = Float32()
        pos_msg.data = status_dict['position_mm']
        self.position_pub.publish(pos_msg)

        # Publish object detection
        obj_msg = Bool()
        obj_msg.data = status_msg.object_detected
        self.object_detected_pub.publish(obj_msg)

    def command_callback(self, msg: GripperCommand) -> None:
        """
        Handle gripper command messages.

        Args:
            msg: GripperCommand message with position, speed, and force
        """
        self.get_logger().debug(f"Received command: pos={msg.position}, speed={msg.speed}, force={msg.force}")

        if not self.check_activated():
            return

        with self.lock:
            if msg.position < 0.0:
                # Negative position means close (special case for compatibility)
                result = self.driver.close(int(msg.speed), int(msg.force))
            else:
                result = self.driver.go_to_mm(msg.position, int(msg.speed), int(msg.force))

        if not result:
            self.get_logger().error("Failed to execute gripper command")

    def position_callback(self, msg: Float32) -> None:
        """
        Handle position command messages.

        Args:
            msg: Float32 message with position in mm
        """
        self.get_logger().debug(f"Received position command: {msg.data} mm")

        if not self.check_activated():
            return

        with self.lock:
            result = self.driver.go_to_mm(msg.data)

        if not result:
            self.get_logger().error("Failed to execute position command")