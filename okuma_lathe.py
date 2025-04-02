#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, Float64, String, Int32
from std_srvs.srv import Trigger
import threading
import time
import clr
import sys
import os
from custom_interfaces.msg import MachineState
from custom_interfaces.srv import ExecuteProgram

# Add references to the Okuma API assemblies
# These paths would need to be adjusted based on actual installation location
OKUMA_API_PATH = os.environ.get('OKUMA_API_PATH', 'C:/Program Files (x86)/Okuma/Open API/')

# Add Okuma API path to the system path
sys.path.append(OKUMA_API_PATH)

# Load Okuma API assemblies using Python.NET
clr.AddReference('Okuma.CLDATAPI.DataAPI')
clr.AddReference('Okuma.CMDATAPI.DataAPI')
clr.AddReference('Okuma.CMCMDAPI.CommandAPI')

# Import Okuma namespaces
from Okuma.CLDATAPI.DataAPI import CNCLathe
from Okuma.CMDATAPI.DataAPI import CMachine
from Okuma.CMCMDAPI.CommandAPI import CNCLathe as CNCLatheCommand


class OkumaLatheInterface(Node):
    """
    ROS2 Node for interfacing with an Okuma CNC lathe using the official Okuma Open API SDK.

    This node provides interfaces for controlling the door, collet, spindle,
    executing CNC programs, and monitoring machine state.
    """

    def __init__(self):
        super().__init__('okuma_lathe_interface')

        # Parameters
        self.declare_parameter('poll_rate', 5.0)  # Hz
        self.declare_parameter('connect_retry_interval', 5.0)  # seconds

        self.poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value
        self.connect_retry_interval = self.get_parameter('connect_retry_interval').get_parameter_value().double_value

        # Initialize connection state
        self.connected = False
        self.lock = threading.Lock()
        self.api_initialized = False

        # API instances
        self.cnc_lathe = None
        self.machine = None
        self.command = None

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
            'lathe/door_state',  # True = open, False = closed
            10
        )

        self.collet_state_publisher = self.create_publisher(
            Bool,
            'lathe/collet_state',  # True = closed, False = open
            10
        )

        self.program_name_publisher = self.create_publisher(
            String,
            'lathe/current_program',
            10
        )

        self.spindle_speed_publisher = self.create_publisher(
            Float64,
            'lathe/spindle_speed',
            10
        )

        self.alarm_publisher = self.create_publisher(
            String,
            'lathe/alarm',
            10
        )

        self.operation_mode_publisher = self.create_publisher(
            String,
            'lathe/operation_mode',
            10
        )

        self.execution_mode_publisher = self.create_publisher(
            String,
            'lathe/execution_mode',
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

        self.cycle_start_service = self.create_service(
            Trigger,
            'lathe/cycle_start',
            self.handle_cycle_start,
            callback_group=cb_group
        )

        self.cycle_stop_service = self.create_service(
            Trigger,
            'lathe/cycle_stop',
            self.handle_cycle_stop,
            callback_group=cb_group
        )

        self.reset_service = self.create_service(
            Trigger,
            'lathe/reset',
            self.handle_reset,
            callback_group=cb_group
        )

        # Create timer for state polling
        self.timer = self.create_timer(
            1.0 / self.poll_rate,
            self.publish_state,
            callback_group=cb_group
        )

        self.get_logger().info("Okuma lathe interface initialized.")

        # Start connection thread
        self.connection_thread = threading.Thread(target=self.connection_worker)
        self.connection_thread.daemon = True
        self.connection_thread.start()

    def initialize_api(self):
        """Initialize the Okuma API instances."""
        try:
            # Initialize API instances
            self.cnc_lathe = CNCLathe()
            self.machine = CMachine()
            self.command = CNCLatheCommand()

            self.api_initialized = True
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Okuma API: {str(e)}")
            self.api_initialized = False
            return False

    def connect(self):
        """Connect to the Okuma lathe using the API."""
        with self.lock:
            if self.connected:
                return True

            if not self.api_initialized and not self.initialize_api():
                return False

            try:
                # Initialize the API connection
                self.cnc_lathe.InitializeClass()
                self.machine.InitializeClass()
                self.command.InitializeClass()

                # Verify connection by attempting to get machine info
                version = self.cnc_lathe.GetLibraryVersion()
                self.get_logger().info(f"Connected to Okuma lathe. API version: {version}")

                self.connected = True
                return True
            except Exception as e:
                self.get_logger().error(f"Failed to connect to Okuma lathe: {str(e)}")
                self.connected = False
                return False

    def disconnect(self):
        """Disconnect from the Okuma lathe API."""
        with self.lock:
            if not self.connected:
                return

            try:
                if self.cnc_lathe:
                    self.cnc_lathe.Close()
                if self.machine:
                    self.machine.Close()
                if self.command:
                    self.command.Close()

                self.connected = False
                self.get_logger().info("Disconnected from Okuma lathe")
            except Exception as e:
                self.get_logger().error(f"Error during disconnection: {str(e)}")

    def connection_worker(self):
        """Background thread to maintain connection to the lathe."""
        while True:
            if not self.connected:
                self.connect()
            time.sleep(self.connect_retry_interval)

    def get_machine_state(self):
        """Get the current state of the machine."""
        with self.lock:
            if not self.connected and not self.connect():
                return None

            try:
                state = MachineState()

                # Get basic machine information
                state.operation_mode = self.machine.GetOperationMode()
                state.execution_mode = self.cnc_lathe.GetExecutionMode()
                state.spindle_speed = self.cnc_lathe.GetActualSpindleSpeed()
                state.feed_rate = self.cnc_lathe.GetActualFeedrate()
                state.rapid_override = self.cnc_lathe.GetRapidOverride()
                state.spindle_override = self.cnc_lathe.GetSpindleOverride()
                state.feedrate_override = self.cnc_lathe.GetFeedrateOverride()

                # Check if any alarms are active
                if self.machine.GetAlarmStatus():
                    state.alarm_active = True
                    # Retrieve first active alarm as an example
                    alarms = self.machine.GetAlarms()
                    if alarms and len(alarms) > 0:
                        state.alarm_message = alarms[0].GetAlarmMessage()
                else:
                    state.alarm_active = False
                    state.alarm_message = ""

                return state
            except Exception as e:
                self.get_logger().error(f"Error getting machine state: {str(e)}")
                self.connected = False
                return None

    def get_door_state(self):
        """Get the current state of the machine door (open/closed)."""
        with self.lock:
            if not self.connected and not self.connect():
                return None

            try:
                # Note: The exact method name may vary based on API version
                # Some versions use IsDoorOpen() instead of GetDoorState()
                try:
                    return self.cnc_lathe.IsDoorOpen()
                except AttributeError:
                    try:
                        return self.cnc_lathe.GetDoorState()
                    except:
                        self.get_logger().warn("Door state method not found in API")
                        return None
            except Exception as e:
                self.get_logger().error(f"Error getting door state: {str(e)}")
                self.connected = False
                return None

    def get_collet_state(self):
        """Get the current state of the collet (open/closed)."""
        with self.lock:
            if not self.connected and not self.connect():
                return None

            try:
                # Check chuck open/close status
                # True = closed, False = open
                try:
                    # Depending on the machine, this might be GetChuckState or GetColletState
                    return self.cnc_lathe.GetChuckClampStatus()
                except AttributeError:
                    try:
                        return self.cnc_lathe.GetColletClampStatus()
                    except:
                        self.get_logger().warn("Collet/Chuck state method not found in API")
                        return None
            except Exception as e:
                self.get_logger().error(f"Error getting collet state: {str(e)}")
                self.connected = False
                return None

    def get_spindle_speed(self):
        """Get the current spindle speed."""
        with self.lock:
            if not self.connected and not self.connect():
                return None

            try:
                return self.cnc_lathe.GetActualSpindleSpeed()
            except Exception as e:
                self.get_logger().error(f"Error getting spindle speed: {str(e)}")
                self.connected = False
                return None

    def get_current_program(self):
        """Get the name of the currently loaded program."""
        with self.lock:
            if not self.connected and not self.connect():
                return None

            try:
                return self.cnc_lathe.GetActiveProgramFileName()
            except Exception as e:
                self.get_logger().error(f"Error getting current program: {str(e)}")
                self.connected = False
                return None

    def open_door(self):
        """Command to open the machine door."""
        with self.lock:
            if not self.connected and not self.connect():
                return False

            try:
                self.command.SetDoor(True)  # True for open
                return True
            except Exception as e:
                self.get_logger().error(f"Error opening door: {str(e)}")
                self.connected = False
                return False

    def close_door(self):
        """Command to close the machine door."""
        with self.lock:
            if not self.connected and not self.connect():
                return False

            try:
                self.command.SetDoor(False)  # False for closed
                return True
            except Exception as e:
                self.get_logger().error(f"Error closing door: {str(e)}")
                self.connected = False
                return False

    def open_collet(self):
        """Command to open the collet/chuck."""
        with self.lock:
            if not self.connected and not self.connect():
                return False

            try:
                # Try different possible methods
                try:
                    self.command.UnclampChuck()
                    return True
                except AttributeError:
                    try:
                        self.command.UnclampCollet()
                        return True
                    except:
                        self.get_logger().error("Collet/Chuck unclamp method not found in API")
                        return False
            except Exception as e:
                self.get_logger().error(f"Error opening collet: {str(e)}")
                self.connected = False
                return False

    def close_collet(self):
        """Command to close the collet/chuck."""
        with self.lock:
            if not self.connected and not self.connect():
                return False

            try:
                # Try different possible methods
                try:
                    self.command.ClampChuck()
                    return True
                except AttributeError:
                    try:
                        self.command.ClampCollet()
                        return True
                    except:
                        self.get_logger().error("Collet/Chuck clamp method not found in API")
                        return False
            except Exception as e:
                self.get_logger().error(f"Error closing collet: {str(e)}")
                self.connected = False
                return False

    def execute_program(self, program_number):
        """Command to execute a specific program on the machine."""
        with self.lock:
            if not self.connected and not self.connect():
                return False

            try:
                # Format program number according to Okuma conventions
                program_name = f"O{program_number}"

                # Select the program
                self.command.SelectMainProgram(program_name)

                # CycleStart will be called separately
                return True
            except Exception as e:
                self.get_logger().error(f"Error selecting program {program_number}: {str(e)}")
                self.connected = False
                return False

    def cycle_start(self):
        """Command to start the machining cycle."""
        with self.lock:
            if not self.connected and not self.connect():
                return False

            try:
                self.command.CycleStart()
                return True
            except Exception as e:
                self.get_logger().error(f"Error starting cycle: {str(e)}")
                self.connected = False
                return False

    def cycle_stop(self):
        """Command to stop the machining cycle."""
        with self.lock:
            if not self.connected and not self.connect():
                return False

            try:
                self.command.CycleStop()
                return True
            except Exception as e:
                self.get_logger().error(f"Error stopping cycle: {str(e)}")
                self.connected = False
                return False

    def reset_machine(self):
        """Command to reset the machine."""
        with self.lock:
            if not self.connected and not self.connect():
                return False

            try:
                self.command.ResetCNC()
                return True
            except Exception as e:
                self.get_logger().error(f"Error resetting machine: {str(e)}")
                self.connected = False
                return False

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

        # Get and publish spindle speed
        spindle_speed = self.get_spindle_speed()
        if spindle_speed is not None:
            self.spindle_speed_publisher.publish(Float64(data=float(spindle_speed)))

        # Get and publish current program
        program = self.get_current_program()
        if program is not None:
            self.program_name_publisher.publish(String(data=program))

        # Publish additional status information if state is available
        if state:
            # Publish operation mode
            self.operation_mode_publisher.publish(String(data=str(state.operation_mode)))

            # Publish execution mode
            self.execution_mode_publisher.publish(String(data=str(state.execution_mode)))

            # Publish alarm if active
            if state.alarm_active:
                self.alarm_publisher.publish(String(data=state.alarm_message))

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
        response.message = f"Program {program_number} selected successfully" if success else f"Failed to select program {program_number}"
        return response

    def handle_cycle_start(self, request, response):
        """Handle cycle start service request."""
        success = self.cycle_start()
        response.success = success
        response.message = "Cycle start command sent successfully" if success else "Failed to send cycle start command"
        return response

    def handle_cycle_stop(self, request, response):
        """Handle cycle stop service request."""
        success = self.cycle_stop()
        response.success = success
        response.message = "Cycle stop command sent successfully" if success else "Failed to send cycle stop command"
        return response

    def handle_reset(self, request, response):
        """Handle reset service request."""
        success = self.reset_machine()
        response.success = success
        response.message = "Reset command sent successfully" if success else "Failed to send reset command"
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