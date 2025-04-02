#!/usr/bin/env python3

import time
import argparse
from onrobot_screwdriver import RS

def run_demo():
    """Runs screwdriver demonstration once."""
    rs = RS(toolchanger_ip, toolchanger_port)
    rs.set_shank_position(0)
    print(rs.get_status())
    rs.get_achieved_torque()
    print('torque', rs.get_achieved_torque())
    rs.drive_screw()
    print(rs.get_status())
    print(rs.get_force_torque())
    rs.close_connection()

if __name__ == '__main__':
    # args = get_options()
    # gripper = args.gripper
    toolchanger_ip = '192.168.1.1'
    toolchanger_port = 502
    run_demo()