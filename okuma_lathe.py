#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import socket
import time
import struct
import threading
from functools import partial
from custom_interfaces.msg import MachineState
from custom_interfaces.srv import ExecuteProgram


class OkumaLatheInterface(Node):
    """
    ROS2 Node for interfacing with an Okuma CNC lathe.

    This node provides interfaces for controlling the door, collet,
    executing CNC programs, and monitoring machine state.

    Communication is implemented using the Okuma THINC API over Ethernet.
    """

    def __init__(self):
        super().__init__('okuma_lathe_interface')

        # Parameters
        self.declare_parameter('lathe_ip', '192.168.1.10')
        self.declare_parameter('lathe_port', 8193)  # Default port for Okuma THINC API
        self.declare_parameter('poll_rate', 5.0)  # Hz
        self.declare_parameter('connection_timeout', 5.0)  # seconds

        self.lathe_ip = self.get_parameter('lathe_ip').get_parameter_value().string_value
        self.lathe_port = self.get_parameter('lathe_port').get_parameter_value().integer_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value

        # Connection state
        self.connected = False
        self.socket = None
        self.lock = threading.Lock()

        # Create callback group for concurrent execution
        cb_group = ReentrantCallbackGroup()

        # Create publishers
        self.state_publisher = self.create_publisher(
            MachineState,
            'lathe/state',
            10
        )

        self.door_state_publisher = self.create_publisher(
            Bool,
            'lathe/door_state',
            10
        )

        self.collet_state_publisher = self.create_publisher(
            Bool,
            'lathe/collet_state',  # True = closed, False = open
            10
        )

        self.program_complete_publisher = self.create_publisher(
            Bool,
            'lathe/program_complete',
            10
        )

        # Create services
        self.open_door_service = self.create_service(
            Trigger,
            'lathe/open_door',
            self.handle_open_door,
            callback_group=cb_group
        )

        self.close_door_service = self.create_service(
            Trigger,
            'lathe/close_door',
            self.handle_close_door,
            callback_group=cb_group
        )

        # Add collet control services
        self.open_collet_service = self.create_service(
            Trigger,
            'lathe/open_collet',
            self.handle_open_collet,
            callback_group=cb_group
        )

        self.close_collet_service = self.create_service(
            Trigger,
            'lathe/close_collet',
            self.handle_close_collet,
            callback_group=cb_group
        )

        self.execute_program_service = self.create_service(
            ExecuteProgram,
            'lathe/execute_program',
            self.handle_execute_program,
            callback_group=cb_group
        )

        # Create timer for state polling
        self.timer = self.create_timer(
            1.0 / self.get_parameter('poll_rate').get_parameter_value().double_value,
            self.publish_state,
            callback_group=cb_group
        )

        self.get_logger().info(f"Okuma lathe interface initialized. "
                               f"Connecting to {self.lathe_ip}:{self.lathe_port}")

        # Initial connection attempt
        self.connect()

    def connect(self):
        """Connect to the Okuma lathe's THINC API."""
        with self.lock:
            if self.connected:
                return True

            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(self.connection_timeout)
                self.socket.connect((self.lathe_ip, self.lathe_port))
                self.connected = True
                self.get_logger().info("Connected to Okuma lathe")
                return True
            except socket.error as e:
                self.get_logger().error(f"Failed to connect to Okuma lathe: {e}")
                self.connected = False
                self.socket = None
                return False

    def disconnect(self):
        """Disconnect from the Okuma lathe's THINC API."""
        with self.lock:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                finally:
                    self.socket = None
                    self.connected = False

    def send_command(self, command_code, data=None):
        """
        Send a command to the Okuma lathe.

        Args:
            command_code (int): The code for the command
            data (bytes, optional): Additional data for the command

        Returns:
            tuple: (success, response_data)
        """
        if data is None:
            data = b''

        # Command format:
        # - 4 bytes: Command code (int)
        # - 4 bytes: Data length (int)
        # - N bytes: Data
        with self.lock:
            if not self.connected:
                if not self.connect():
                    return False, None

            try:
                # Prepare command packet
                packet = struct.pack('>II', command_code, len(data)) + data

                # Send command
                self.socket.sendall(packet)

                # Receive response
                # - 4 bytes: Status code (int)
                # - 4 bytes: Data length (int)
                # - N bytes: Data
                header = self.socket.recv(8)
                status, data_length = struct.unpack('>II', header)

                response_data = b''
                if data_length > 0:
                    response_data = self.socket.recv(data_length)

                return status == 0, response_data

            except socket.error as e:
                self.get_logger().error(f"Communication error with lathe: {e}")
                self.connected = False
                self.socket = None
                return False, None

    # Okuma THINC API command codes (hypothetical - would need to match actual API)
    CMD_GET_STATE = 0x0001
    CMD_GET_DOOR_STATE = 0x0002
    CMD_OPEN_DOOR = 0x0003
    CMD_CLOSE_DOOR = 0x0004
    CMD_EXECUTE_PROGRAM = 0x0005
    CMD_GET_PROGRAM_STATE = 0x0006

    # New collet command codes
    CMD_GET_COLLET_STATE = 0x0007
    CMD_OPEN_COLLET = 0x0008
    CMD_CLOSE_COLLET = 0x0009

    def get_machine_state(self):
        """Get the current state of the machine."""
        success, data = self.send_command(self.CMD_GET_STATE)
        if not success or not data:
            return None

        # Parse machine state data
        # Format depends on actual API, this is a placeholder
        try:
            # Example state structure:
            # - Byte 0: Machine mode (0=auto, 1=manual, etc.)
            # - Byte 1: Error code
            # - Byte 2-5: Current spindle speed (float)
            # - Byte 6-9: Feed rate (float)
            mode, error = struct.unpack('>BB', data[0:2])
            spindle_speed, feed_rate = struct.unpack('>ff', data[2:10])

            state = MachineState()
            state.mode = mode
            state.error_code = error
            state.spindle_speed = spindle_speed
            state.feed_rate = feed_rate

            return state

        except struct.error as e:
            self.get_logger().error(f"Error parsing machine state: {e}")
            return None

    def get_door_state(self):
        """Get the current state of the machine door (open/closed)."""
        success, data = self.send_command(self.CMD_GET_DOOR_STATE)
        if not success or not data:
            return None

        # Parse door state
        # Assuming 1 byte response: 0 = closed, 1 = open
        try:
            door_state = struct.unpack('>B', data)[0]
            return bool(door_state)
        except struct.error as e:
            self.get_logger().error(f"Error parsing door state: {e}")
            return None

    def get_collet_state(self):
        """Get the current state of the collet (open/closed)."""
        success, data = self.send_command(self.CMD_GET_COLLET_STATE)
        if not success or not data:
            return None

        # Parse collet state
        # Assuming 1 byte response: 0 = open, 1 = closed
        try:
            collet_state = struct.unpack('>B', data)[0]
            return bool(collet_state)  # True = closed, False = open
        except struct.error as e:
            self.get_logger().error(f"Error parsing collet state: {e}")
            return None

    def get_program_state(self):
        """Check if the current program is complete."""
        success, data = self.send_command(self.CMD_GET_PROGRAM_STATE)
        if not success or not data:
            return None

        # Parse program state
        # Assuming 1 byte response: 0 = running, 1 = complete, 2 = error
        try:
            program_state = struct.unpack('>B', data)[0]
            return program_state == 1  # True if complete
        except struct.error as e:
            self.get_logger().error(f"Error parsing program state: {e}")
            return None

    def open_door(self):
        """Command to open the machine door."""
        success, _ = self.send_command(self.CMD_OPEN_DOOR)
        return success

    def close_door(self):
        """Command to close the machine door."""
        success, _ = self.send_command(self.CMD_CLOSE_DOOR)
        return success

    def open_collet(self):
        """Command to open the collet."""
        success, _ = self.send_command(self.CMD_OPEN_COLLET)
        return success

    def close_collet(self):
        """Command to close the collet."""
        success, _ = self.send_command(self.CMD_CLOSE_COLLET)
        return success

    def execute_program(self, program_number):
        """Command to execute a specific program on the machine."""
        # Convert program number to bytes and send command
        data = struct.pack('>I', program_number)
        success, _ = self.send_command(self.CMD_EXECUTE_PROGRAM, data)
        return success

    def publish_state(self):
        """Publish the current state of the lathe."""
        # Get and publish machine state
        state = self.get_machine_state()
        if state:
            self.state_publisher.publish(state)

        # Get and publish door state
        door_state = self.get_door_state()
        if door_state is not None:
            self.door_state_publisher.publish(Bool(data=door_state))

        # Get and publish collet state
        collet_state = self.get_collet_state()
        if collet_state is not None:
            self.collet_state_publisher.publish(Bool(data=collet_state))

        # Get and publish program complete status
        program_complete = self.get_program_state()
        if program_complete is not None:
            self.program_complete_publisher.publish(Bool(data=program_complete))

    def handle_open_door(self, request, response):
        """Handle open door service request."""
        success = self.open_door()
        response.success = success
        response.message = "Door open command sent successfully" if success else "Failed to send door open command"
        return response

    def handle_close_door(self, request, response):
        """Handle close door service request."""
        success = self.close_door()
        response.success = success
        response.message = "Door close command sent successfully" if success else "Failed to send door close command"
        return response

    def handle_open_collet(self, request, response):
        """Handle open collet service request."""
        success = self.open_collet()
        response.success = success
        response.message = "Collet open command sent successfully" if success else "Failed to send collet open command"
        return response

    def handle_close_collet(self, request, response):
        """Handle close collet service request."""
        success = self.close_collet()
        response.success = success
        response.message = "Collet close command sent successfully" if success else "Failed to send collet close command"
        return response

    def handle_execute_program(self, request, response):
        """Handle execute program service request."""
        program_number = request.program_number
        success = self.execute_program(program_number)
        response.success = success
        response.message = f"Program {program_number} execution started" if success else f"Failed to start program {program_number}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OkumaLatheInterface()

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