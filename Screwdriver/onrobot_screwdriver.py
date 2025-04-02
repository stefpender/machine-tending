# -----------------------------------------------------------------------------------------
# Title: Controller for Onrobot Screwdriver
# Description: Controlling Onrobot screwriver using MODBUS TCP, must be used with Compute Box
# Author: Stefanie Pender
# Date: April 2, 2024
# -----------------------------------------------------------------------------------------
from pymodbus.client.tcp import ModbusTcpClient as ModbusClient
from pymodbus.exceptions import ModbusIOException

class RS():

    def __init__(self, ip, port):
        self.client = ModbusClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1
            )
        self.screw_length_mm = 25
        self.driver_torque = 100
        self.z_force = 18
        self.open_connection()

    def open_connection(self):
        """Opens the connection with a gripper."""
        self.client.connect()

    def close_connection(self):
        """Closes the connection with the gripper."""
        self.client.close()

    def set_shank_position(self, position):
        """Writes the target shank position for the screwdriver.
        Args:
            position (int): The target shank position in millimeters, within the range 0-55.
        Raises:
            ValueError: If the provided position is out of the valid range.
        """
        # Ensure the provided position is within the valid range
        if not 0 <= position <= 55:
            raise ValueError("Position value out of range. Must be between 0 and 55.")

        # The target shank position is within the range, so we write it to the register
        try:
            write_result = self.client.write_registers(0x0000, [0, position], slave=65)
            if write_result.isError():
                print("Error writing the shank position.")
            else:
                print(f"Shank position set to {position} mm.")
        except ModbusIOException as e:
            print(f"Error in Modbus communication: {e}")

    def get_shank_pos(self):
        """Reads the current shank/Z Axes Position.
        Returns:
            float: The current shank position in millimeters, or None if an error occurs.
        """
        try:
            result = self.client.read_holding_registers(address=258, count=1, slave=65)
            if not result.isError():
                # Assuming the register value is in millimeters, as per your documentation
                shank_pos_mm = result.registers[0]/1000
                print(f"Shank/Z Axes Position is {shank_pos_mm} mm.")
                return shank_pos_mm
            else:
                print("Error reading the shank/Z Axes Position.")
                return None
        except ModbusIOException as e:
            print(f"Error in Modbus communication while reading shank position: {e}")
            return None

    def get_achieved_torque(self):
        """Indicates latest achieved torque in mNm after a tighten command
        """
        result = self.client.read_holding_registers(
            address=260, count=1, slave=65)
        torque = result.registers[0]
        return torque

    def get_status(self):
        """Reads current device status.
        This status field indicates the status of the gripper and its motion.
        It is composed of numerous flags, described in the table below.
        Bit      Name            Description
        1:       busy            screwdriver busy
        2:       Z busy          Z-axis busy
        4:       safety          Error: Z-axis safety activated
        8:       calibration     Error: not calibrated
        16-512:  see manual      Not integrated yet.
        """
        # address   : register number
        # count     : number of registers to be read
        # unit      : slave device address
        try:
            result = self.client.read_holding_registers(
                address=256, count=1, slave=65)
            if not result.isError():
                print("Registers:", result.registers)
                status = format(result.registers[0], '016b')
                # process status as needed
            else:
                # Handle MODBUS error (e.g., log it, retry, etc.)
                print("MODBUS error:", result)
        except AttributeError as e:
            # This catches cases where `result` does not have the expected attributes, including `isError`
            print(f"An unexpected error occurred: {e}")
        # status = format(result.registers[0], '016b')
        status_list = [0] * 4
        if int(status[-1]):
            print("Screwdriver busy")
            status_list[0] = 1
        if int(status[-2]):
            print("Z-axis busy")
            status_list[1] = 1
        if int(status[-3]):
            print("Safety switch z-axis.")
            status_list[2] = 1
        if int(status[-4]):
            print("Not calibrated.")
            status_list[3] = 1
        return status_list

    def set_control_mode(self, command):
        """The control field is used to start and stop screwdriver activity.
        Only one option should be set at a time.
        Please note that the screwdriver will not start a new motion
        before the one currently being executed is done
        (see busy flag in the Status field).
        If z_axis_force is not set, screwdriver will turn in place
        The valid flags are:
        0 (0x0001):  Tighten screw
        1 (0x0001):  Loosen Screw
        2 (0x0002):  Pick up screw
        4 (0x0004):  Stop
        """
        result = self.client.write_register(
            address=4, value=command, slave=65)

    def z_axis_force(self):
        """Target Z axes force to be achieved and maintained.
        In Newtons valid range between 18-30
        """
        result = self.client.write_register(
            address = 1, value = self.z_force, slave=65
        )

    def screw_length(self):
        """Screw/screwing/unscrewing length.
        User inputs in mm - multiplied by 1000 for micrometers
        0 to 35000 micrometers
        """
        result = self.client.write_register(
            address = 2, value = self.screw_length_mm * 1000, slave = 65
        )

    def torque(self):
        """Output torque the screwdriver will try to reach
        Provided in miliNewtons meters 100-5000
        """
        result = self.client.write_register(
            address=3, value = self.driver_torque, slave = 65)

    def drive_screw(self):
        """tighen screw."""
        params = [self.z_force, self.screw_length_mm, self.driver_torque]
        print("Start driving screw.")
        result = self.client.write_registers(
            address=0, values=params, slave=65)

# need to add loosen_screw, pick_up screw at later time.

# Force Torque sensor
    def get_force_torque(self):
        """Force value along the X, Y, Z axis (in the sensor coordinate system) in 1/10 N.  Value is signed int
           Torque value about the X, Y, Z axis (in the sensor coordinate system) in 1/100 Nm
        """

        # Assuming Fx, Fy, and Fz are stored in registers 259, 260, and 261 respectively
        # and they are 16-bit registers. Modbus function code 3 is used for reading
        # holding registers.

        response_fx = self.client.read_holding_registers(259, 1, slave=65)  # Reading Fx
        response_fy = self.client.read_holding_registers(260, 1, slave=65)  # Reading Fy
        response_fz = self.client.read_holding_registers(261, 1, slave=65)  # Reading Fz

        if not (response_fx.isError() or response_fy.isError() or response_fz.isError()):
            fx = response_fx.registers[0]
            fy = response_fy.registers[0]
            fz = response_fz.registers[0]
            print(f"Fx: {fx / 10.0} N, Fy: {fy / 10.0} N, Fz: {fz / 10.0} N")
        else:
            print("Error reading force values")


        response_tx = self.client.read_holding_registers(262, 1, slave=65)  # Reading Tx
        response_ty = self.client.read_holding_registers(263, 1, slave=65)  # Reading Ty
        response_tz = self.client.read_holding_registers(264, 1, slave=65)  # Reading Tz

        if not (response_tx.isError() or response_ty.isError() or response_tz.isError()):
            tx = response_tx.registers[0]
            ty = response_ty.registers[0]
            tz = response_tz.registers[0]
            print(f"Tx: {tx / 100.0} Nm, Ty: {ty / 100.0} Nm, Tz: {tz / 100.0} Nm")
        else:
            print("Error reading torque values")