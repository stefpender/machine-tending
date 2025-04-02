#!/usr/bin/env python3
"""
The Robotiq Hand-E C10 Gripper driver implementation for ROS 2.
This module provides the low-level communication with the gripper via Modbus/TCP.
"""

import time
from enum import Enum
from typing import Optional, Tuple, Dict, Any, List

from pymodbus.client import ModbusTcpClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadBuilder


class GripperStatus(Enum):
    """Gripper status values from gSTA bits."""
    RESET = 0x00
    ACTIVATING = 0x01
    # 0x02 is not used according to manual
    ACTIVE = 0x03


class ObjectStatus(Enum):
    """Object detection status values from gOBJ bits."""
    MOVING = 0x00
    OBJECT_DETECTED_OPENING = 0x01
    OBJECT_DETECTED_CLOSING = 0x02
    AT_REQUESTED_POSITION = 0x03


class FaultStatus(Enum):
    """Fault status values from gFLT bits."""
    NO_FAULT = 0x00
    ACTION_DELAYED = 0x05
    ACTIVATION_NEEDED = 0x07
    MAX_TEMP_EXCEEDED = 0x08
    NO_COMMUNICATION = 0x09
    UNDER_VOLTAGE = 0x0A
    AUTO_RELEASE_IN_PROGRESS = 0x0B
    INTERNAL_FAULT = 0x0C
    ACTIVATION_FAULT = 0x0D
    OVERCURRENT = 0x0E
    AUTO_RELEASE_COMPLETED = 0x0F


class RobotiqHandEDriver:
    """
    Driver for the Robotiq Hand-E C10 Gripper using Modbus/TCP.

    This driver is based on the Robotiq Hand-E C10 Gripper manual,
    adapted to use Modbus/TCP instead of the original Modbus RTU over RS485.

    Registers:
    - Input registers (status): start at 0x07D0 (2000)
    - Output registers (commands): start at 0x03E8 (1000)
    """
    # Register addresses
    INPUT_REGISTERS_START = 0x07D0  # 2000 in decimal
    OUTPUT_REGISTERS_START = 0x03E8  # 1000 in decimal

    # Maximum values
    MAX_POSITION = 255  # Fully closed
    MAX_SPEED = 255  # Maximum speed
    MAX_FORCE = 255  # Maximum force

    def __init__(self, ip_address: str, port: int = 502, slave_id: int = 9,
                 debug: bool = False, logger=None):
        """
        Initialize the gripper driver.

        Args:
            ip_address: IP address of the Modbus/TCP device
            port: Modbus/TCP port (default: 502)
            slave_id: Modbus slave ID (default: 9 as per manual)
            debug: Enable debug prints
            logger: Logger instance for logging (if None, print statements are used)
        """
        self.client = ModbusTcpClient(ip_address, port=port)
        self.slave_id = slave_id
        self.debug = debug
        self.logger = logger
        self.connected = False

    def log(self, message: str, level: str = "info") -> None:
        """Log a message using the provided logger or print."""
        if self.logger:
            if level == "info":
                self.logger.info(message)
            elif level == "warning":
                self.logger.warning(message)
            elif level == "error":
                self.logger.error(message)
            elif level == "debug" and self.debug:
                self.logger.debug(message)
        elif self.debug or level != "debug":
            print(f"[{level.upper()}] {message}")

    def connect(self) -> bool:
        """Connect to the gripper."""
        if self.connected:
            return True

        self.connected = self.client.connect()
        self.log(f"Connected: {self.connected}")
        return self.connected

    def disconnect(self) -> None:
        """Disconnect from the gripper."""
        self.client.close()
        self.connected = False
        self.log("Disconnected")

    def _read_input_registers(self, count: int = 6) -> Optional[List[int]]:
        """
        Read input registers (status registers).

        Args:
            count: Number of registers to read (default: 6)

        Returns:
            List of register values or None if failed
        """
        if not self.connected and not self.connect():
            self.log("Failed to connect", "error")
            return None

        response = self.client.read_input_registers(
            self.INPUT_REGISTERS_START,
            count,
            slave=self.slave_id
        )

        if response.isError():
            self.log(f"Error reading input registers: {response}", "error")
            return None

        return response.registers

    def _write_output_registers(self, values: List[int]) -> bool:
        """
        Write to output registers (command registers).

        Args:
            values: List of values to write to registers

        Returns:
            True if successful, False otherwise
        """
        if not self.connected and not self.connect():
            self.log("Failed to connect", "error")
            return False

        # Write multiple registers
        response = self.client.write_registers(
            self.OUTPUT_REGISTERS_START,
            values,
            slave=self.slave_id
        )

        if response.isError():
            self.log(f"Error writing output registers: {response}", "error")
            return False

        return True

    def get_status(self) -> Optional[Dict[str, Any]]:
        """
        Get the current status of the gripper.

        Returns:
            Dictionary with status information or None if failed
        """
        registers = self._read_input_registers(6)
        if not registers:
            return None

        gripper_status_reg = registers[0]
        fault_status_reg = registers[2]
        position_echo_reg = registers[3]
        position_reg = registers[4]
        current_reg = registers[5]

        # Extract gACT, gGTO, gSTA, gOBJ from gripper status register
        gACT = (gripper_status_reg & 0x01)
        gGTO = (gripper_status_reg & 0x08) >> 3
        gSTA = (gripper_status_reg & 0x30) >> 4
        gOBJ = (gripper_status_reg & 0xC0) >> 6

        # Extract gFLT from fault status register
        gFLT = fault_status_reg & 0x0F

        status = {
            'activated': bool(gACT),
            'goto_active': bool(gGTO),
            'gripper_status': GripperStatus(gSTA).name if gSTA in [status.value for status in
                                                                   GripperStatus] else f"UNKNOWN_{gSTA}",
            'object_status': ObjectStatus(gOBJ).name if gOBJ in [status.value for status in
                                                                 ObjectStatus] else f"UNKNOWN_{gOBJ}",
            'fault_status': FaultStatus(gFLT).name if gFLT in [status.value for status in
                                                               FaultStatus] else f"UNKNOWN_{gFLT}",
            'position_request': position_echo_reg,
            'position': position_reg,
            'current': current_reg * 10,  # Current in mA (value * 10)
            'position_mm': self._position_to_mm(position_reg),
            'open_percentage': 100 - (position_reg / 255 * 100)
        }

        if self.debug:
            self.log(f"Status: {status}", "debug")

        return status

    def _position_to_mm(self, position: int) -> float:
        """
        Convert position value (0-255) to millimeters.

        According to the manual, 0 corresponds to fully open (40mm),
        and 255 to fully closed (0mm), with a quasi-linear relationship.

        Args:
            position: Position value (0-255)

        Returns:
            Opening in millimeters
        """
        return 40 - (position / 255 * 40)

    def _mm_to_position(self, mm: float) -> int:
        """
        Convert millimeters to position value (0-255).

        Args:
            mm: Opening in millimeters (0-40)

        Returns:
            Position value (0-255)
        """
        position = int((40 - mm) / 40 * 255)
        return max(0, min(255, position))

    def reset(self) -> bool:
        """
        Reset the gripper (clear rACT bit).

        Returns:
            True if successful, False otherwise
        """
        return self._write_output_registers([0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def activate(self) -> bool:
        """
        Activate the gripper (set rACT bit).

        Returns:
            True if successful, False otherwise
        """
        return self._write_output_registers([0x01, 0x00, 0x00, 0x00, 0x00, 0x00])

    def activate_and_wait(self, timeout: float = 5.0) -> bool:
        """
        Activate the gripper and wait for activation to complete.

        Args:
            timeout: Timeout in seconds

        Returns:
            True if activation completed successfully, False otherwise
        """
        if not self.activate():
            return False

        start_time = time.time()
        while time.time() - start_time < timeout:
            status = self.get_status()
            if not status:
                return False

            if status['gripper_status'] == GripperStatus.ACTIVE.name:
                return True

            # Check for activation fault
            if status['fault_status'] != FaultStatus.NO_FAULT.name:
                self.log(f"Activation fault: {status['fault_status']}", "error")
                return False

            time.sleep(0.1)

        self.log("Activation timeout", "error")
        return False

    def go_to(self, position: int, speed: int, force: int) -> bool:
        """
        Command the gripper to go to a specified position with specified speed and force.

        Args:
            position: Position (0-255, 0 = fully open, 255 = fully closed)
            speed: Speed (0-255)
            force: Force (0-255)

        Returns:
            True if command was sent successfully, False otherwise
        """
        # Clamp values to valid ranges
        position = max(0, min(self.MAX_POSITION, position))
        speed = max(0, min(self.MAX_SPEED, speed))
        force = max(0, min(self.MAX_FORCE, force))

        # Set rGTO bit and parameters
        cmd = 0x09  # rACT = 1, rGTO = 1

        return self._write_output_registers([cmd, 0x00, 0x00, position, speed, force])

    def open(self, speed: int = 255, force: int = 255) -> bool:
        """
        Open the gripper fully.

        Args:
            speed: Speed (0-255)
            force: Force (0-255)

        Returns:
            True if command was sent successfully, False otherwise
        """
        return self.go_to(0, speed, force)

    def close(self, speed: int = 255, force: int = 255) -> bool:
        """
        Close the gripper fully.

        Args:
            speed: Speed (0-255)
            force: Force (0-255)

        Returns:
            True if command was sent successfully, False otherwise
        """
        return self.go_to(255, speed, force)

    def go_to_mm(self, mm: float, speed: int = 255, force: int = 255) -> bool:
        """
        Command the gripper to go to a specified position in millimeters.

        Args:
            mm: Opening in millimeters (0-40)
            speed: Speed (0-255)
            force: Force (0-255)

        Returns:
            True if command was sent successfully, False otherwise
        """
        position = self._mm_to_position(mm)
        return self.go_to(position, speed, force)

    def go_to_percent(self, percent: float, speed: int = 255, force: int = 255) -> bool:
        """
        Command the gripper to go to a specified position as a percentage of the stroke.

        Args:
            percent: Percentage open (0-100, 0 = fully closed, 100 = fully open)
            speed: Speed (0-255)
            force: Force (0-255)

        Returns:
            True if command was sent successfully, False otherwise
        """
        # Convert percentage to position value
        # 0% = fully closed (255), 100% = fully open (0)
        position = int((100 - percent) / 100 * 255)
        position = max(0, min(255, position))

        return self.go_to(position, speed, force)

    def wait_for_motion_complete(self, timeout: float = 5.0) -> Tuple[bool, Dict[str, Any]]:
        """
        Wait for the current motion to complete.

        Args:
            timeout: Timeout in seconds

        Returns:
            Tuple of (success, status)
        """
        start_time = time.time()
        last_position = None
        stability_counter = 0

        while time.time() - start_time < timeout:
            status = self.get_status()
            if not status:
                return False, {}

            # Check if motion is complete based on object status
            if status['goto_active']:
                if status['object_status'] in [ObjectStatus.OBJECT_DETECTED_OPENING.name,
                                               ObjectStatus.OBJECT_DETECTED_CLOSING.name,
                                               ObjectStatus.AT_REQUESTED_POSITION.name]:
                    return True, status

                # Check if position has stabilized (not moving anymore)
                current_position = status['position']
                if last_position == current_position:
                    # Position hasn't changed, might be complete
                    stability_counter += 1
                    if stability_counter >= 3:  # 3 consecutive stable readings
                        return True, status
                else:
                    stability_counter = 0

                last_position = current_position
            else:
                # GTO not active, motion not in progress
                return True, status

            time.sleep(0.1)

        self.log("Motion timeout", "warning")
        return False, status

    def grasp_object(self, speed: int = 255, force: int = 255, timeout: float = 5.0) -> Tuple[bool, Dict[str, Any]]:
        """
        Close the gripper to grasp an object and wait for object detection.

        Args:
            speed: Speed (0-255)
            force: Force (0-255)
            timeout: Timeout in seconds

        Returns:
            Tuple of (success, status)
        """
        if not self.close(speed, force):
            return False, {}

        # Wait for motion to complete or object detection
        success, status = self.wait_for_motion_complete(timeout)

        if success:
            # Check if object was detected
            object_detected = status['object_status'] == ObjectStatus.OBJECT_DETECTED_CLOSING.name
            return object_detected, status

        return False, status

    def release_object(self, speed: int = 255, force: int = 255, timeout: float = 5.0) -> Tuple[bool, Dict[str, Any]]:
        """
        Open the gripper to release an object and wait for completion.

        Args:
            speed: Speed (0-255)
            force: Force (0-255)
            timeout: Timeout in seconds

        Returns:
            Tuple of (success, status)
        """
        if not self.open(speed, force):
            return False, {}

        # Wait for motion to complete
        return self.wait_for_motion_complete(timeout)

    def emergency_release(self, open_direction: bool = True) -> bool:
        """
        Perform emergency auto-release (slowly opens or closes the gripper).

        Args:
            open_direction: True for opening direction, False for closing direction

        Returns:
            True if command was sent successfully, False otherwise
        """
        # Set rATR bit and optionally rARD bit
        cmd = 0x01  # rACT = 1
        cmd |= 0x04  # rATR = 1

        if open_direction:
            cmd |= 0x02  # rARD = 1

        return self._write_output_registers([cmd, 0x00, 0x00, 0x00, 0x00, 0x00])

    def is_object_detected(self) -> bool:
        """
        Check if an object is currently detected by the gripper.

        Returns:
            True if object is detected, False otherwise
        """
        status = self.get_status()
        if not status:
            return False

        return status['object_status'] in [
            ObjectStatus.OBJECT_DETECTED_OPENING.name,
            ObjectStatus.OBJECT_DETECTED_CLOSING.name
        ]

    def get_fault_message(self) -> str:
        """
        Get a human-readable message for the current fault status.

        Returns:
            Fault message string
        """
        status = self.get_status()
        if not status:
            return "Unable to get status"

        fault_status = status['fault_status']

        fault_messages = {
            FaultStatus.NO_FAULT.name: "No fault",
            FaultStatus.ACTION_DELAYED.name: "Action delayed; activation must be completed first",
            FaultStatus.ACTIVATION_NEEDED.name: "Activation bit must be set before action",
            FaultStatus.MAX_TEMP_EXCEEDED.name: "Maximum operating temperature exceeded (≥ 85°C)",
            FaultStatus.NO_COMMUNICATION.name: "No communication during at least 1 second",
            FaultStatus.UNDER_VOLTAGE.name: "Under minimum operating voltage",
            FaultStatus.AUTO_RELEASE_IN_PROGRESS.name: "Automatic release in progress",
            FaultStatus.INTERNAL_FAULT.name: "Internal fault, contact support",
            FaultStatus.ACTIVATION_FAULT.name: "Activation fault, check for interference",
            FaultStatus.OVERCURRENT.name: "Overcurrent triggered",
            FaultStatus.AUTO_RELEASE_COMPLETED.name: "Automatic release completed"
        }

        return fault_messages.get(fault_status, f"Unknown fault: {fault_status}")