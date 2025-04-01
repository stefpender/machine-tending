#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
from std_msgs.msg import String
import pymodbus.client as ModbusClient
from pymodbus.exceptions import ModbusException
from custom_interfaces.action import GripperControl
from custom_interfaces.msg import GripperState


class RobotiqGripperInterface(Node):
    """
    ROS2 Node for interfacing with a Robotiq Hand-E C10 three-finger gripper
    over Modbus/TCP.
    """

    # Modbus Register Addresses
    ACTION_REQUEST_REGISTER = 0x03E8  # Register for sending commands
    GRIPPER_STATUS_REGISTER = 0x07D0  # Register for reading status

    # Command Bits
    ACTIVATE_GRIPPER = 0x0100
    RESET_GRIPPER = 0x0200
    SET_POSITION = 0x0900

    # Status Bits
    STATUS_INITIALIZED = 0x01
    STATUS_OBJECT_DETECTED = 0x02
    STATUS_MOVING = 0x04
    STATUS_AT_REQUESTED_POSITION = 0x08

    def __init__(self):
        super().__init__('robotiq_gripper_interface')

        # Parameters
        self.declare_parameter('modbus_ip', '192.168.1.11')
        self.declare_parameter('modbus_port', 502)
        self.declare_parameter('node_rate', 10.0)  # Hz

        self.modbus_ip = self.get_parameter('modbus_ip').get_parameter_value().string_value
        self.modbus_port = self.get_parameter('modbus_port').get_parameter_value().integer_value

        # Initialize Modbus client
        self.client = ModbusClient.ModbusTcpClient(
            host=self.modbus_ip,
            port=self.modbus_port
        )

        # Callback group for allowing concurrent execution
        cb_group = ReentrantCallbackGroup()

        # Create publishers
        self.state_publisher = self.create_publisher(
            GripperState,
            'gripper/state',
            10
        )

        # Create services
        self.activate_service = self.create_service(
            Trigger,
            'gripper/activate',
            self.handle_activate,
            callback_group=cb_group
        )

        self.reset_service = self.create_service(
            Trigger,
            'gripper/reset',
            self.handle_reset,
            callback_group=cb_group
        )

        # Create action server
        self._action_server = ActionServer(
            self,
            GripperControl,
            'gripper/control',
            self.execute_gripper_control,
            callback_group=cb_group
        )

        # Create timer for state publishing
        self.timer = self.create_timer(
            1.0 / self.get_parameter('node_rate').get_parameter_value().double_value,
            self.publish_state,
            callback_group=cb_group
        )

        self.get_logger().info(f"Robotiq gripper interface initialized. "
                               f"Connecting to {self.modbus_ip}:{self.modbus_port}")

    def connect(self):
        """Connect to the Modbus TCP server."""
        try:
            self.client.connect()
            return True
        except ModbusException as e:
            self.get_logger().error(f"Failed to connect to gripper: {e}")
            return False

    def disconnect(self):
        """Disconnect from the Modbus TCP server."""
        self.client.close()

    def send_command(self, command):
        """Send a command to the gripper via Modbus."""
        try:
            result = self.client.write_register(
                self.ACTION_REQUEST_REGISTER,
                command
            )
            return not result.isError()
        except ModbusException as e:
            self.get_logger().error(f"Failed to send command to gripper: {e}")
            return False

    def read_status(self):
        """Read the current status of the gripper."""
        try:
            result = self.client.read_holding_registers(
                self.GRIPPER_STATUS_REGISTER,
                2
            )
            if result.isError():
                self.get_logger().error("Error reading gripper status")
                return None

            # Extract status information from registers
            status_reg = result.registers[0]
            position = result.registers[1]

            # Create and populate status message
            status = GripperState()
            status.is_initialized = bool(status_reg & self.STATUS_INITIALIZED)
            status.is_object_detected = bool(status_reg & self.STATUS_OBJECT_DETECTED)
            status.is_moving = bool(status_reg & self.STATUS_MOVING)
            status.at_requested_position = bool(status_reg & self.STATUS_AT_REQUESTED_POSITION)
            status.position = float(position) / 255.0  # Normalize position to 0.0-1.0

            return status

        except ModbusException as e:
            self.get_logger().error(f"Failed to read gripper status: {e}")
            return None

    def publish_state(self):
        """Publish the current state of the gripper."""
        if not self.client.is_socket_open():
            if not self.connect():
                return

        status = self.read_status()
        if status:
            self.state_publisher.publish(status)

    def handle_activate(self, request, response):
        """Handle activation service request."""
        success = self.send_command(self.ACTIVATE_GRIPPER)
        response.success = success
        response.message = "Gripper activation succeeded" if success else "Gripper activation failed"
        return response

    def handle_reset(self, request, response):
        """Handle reset service request."""
        success = self.send_command(self.RESET_GRIPPER)
        response.success = success
        response.message = "Gripper reset succeeded" if success else "Gripper reset failed"
        return response

    def execute_gripper_control(self, goal_handle):
        """Execute gripper control action."""
        self.get_logger().info('Executing gripper control...')

        goal = goal_handle.request
        position = goal.position  # 0.0 = fully closed, 1.0 = fully open
        force = goal.force  # 0.0 = minimum force, 1.0 = maximum force

        # Convert position and force to gripper-specific values (0-255)
        position_value = int(position * 255)
        force_value = int(force * 255)

        # Combine position and force into command
        # High byte = position, Low byte = force
        command = self.SET_POSITION | (position_value << 8) | force_value

        # Send command to gripper
        if not self.send_command(command):
            goal_handle.abort()
            return GripperControl.Result(success=False)

        # Create feedback and result messages
        feedback = GripperControl.Feedback()
        result = GripperControl.Result()

        # Wait for gripper to complete movement or timeout
        rate = self.create_rate(10)  # 10 Hz
        timeout_counter = 0
        max_timeout = 50  # 5 seconds timeout

        while rclpy.ok() and timeout_counter < max_timeout:
            status = self.read_status()

            if status is None:
                # Connection error
                goal_handle.abort()
                return GripperControl.Result(success=False)

            # Publish feedback
            feedback.current_position = status.position
            feedback.is_moving = status.is_moving
            feedback.object_detected = status.is_object_detected
            goal_handle.publish_feedback(feedback)

            # Check if gripper has reached requested position or detected an object
            if status.at_requested_position or status.is_object_detected:
                break

            timeout_counter += 1
            rate.sleep()

        # Set result
        result.success = True
        result.final_position = feedback.current_position
        result.object_detected = feedback.object_detected

        # Check for timeout
        if timeout_counter >= max_timeout:
            self.get_logger().warn("Gripper control action timed out")
            result.success = False

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotiqGripperInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()