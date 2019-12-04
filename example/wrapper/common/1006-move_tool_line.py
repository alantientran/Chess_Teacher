#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move Tool Line
"""

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI


#######################################################
"""
Just for test example
"""
if len(sys.argv) >= 2:
    ip = sys.argv[1]
else:
    try:
        from configparser import ConfigParser
        parser = ConfigParser()
        parser.read('../robot.conf')
        ip = parser.get('xArm', 'ip')
    except:
        ip = input('Please input the xArm ip address:')
        if not ip:
            print('input error, exit')
            sys.exit(1)
########################################################


arm = XArmAPI(ip, is_radian=True)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

arm.set_tool_position(x=100, y=0, z=0, roll=0, pitch=0, yaw=0, speed=100, is_radian=False, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_tool_position(x=0, y=200, z=0, roll=0, pitch=0, yaw=0, speed=200, is_radian=False, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_tool_position(x=200, y=0, z=0, roll=0, pitch=0, yaw=0, speed=300, is_radian=False, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_tool_position(x=0, y=-400, z=0, roll=0, pitch=0, yaw=0, speed=400, is_radian=False, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_tool_position(x=-200, y=0, z=0, roll=0, pitch=0, yaw=0, speed=500, is_radian=False, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_tool_position(x=0, y=200, z=0, roll=0, pitch=0, yaw=0, speed=600, is_radian=False, wait=True)
print(arm.get_position(), arm.get_position(is_radian=False))

arm.reset(wait=True)
arm.disconnect()
