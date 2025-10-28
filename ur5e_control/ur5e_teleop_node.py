#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5e Teleoperation Node
Subscribes to /teleoperation/joint_angles topic and sends arm joint positions to UR5e via RTDE
"""

import sys
import os
import rospy
import math
import logging
from sensor_msgs.msg import JointState

# Add RTDE library path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'RTDE_Python_Client_Library'))

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

# Configure logging
logging.basicConfig(level=logging.INFO)
_log = logging.getLogger("UR5e_Teleop")

# Robot configuration
ROBOT_HOST = "192.168.1.101"  # UR5e robot IP
ROBOT_PORT = 30004

# Config file path
CONFIG_FILE = os.path.join(os.path.dirname(__file__), '..', 'config', 'ur5e_control_config.xml')

# Joint names for UR5e arm
ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]


class UR5eTeleopNode:
    def __init__(self):
        rospy.init_node('ur5e_teleop_node', anonymous=True)
        _log.info("Initializing UR5e Teleop Node...")
        
        # Initialize RTDE connection
        self.rtde_connected = False
        self.con = None
        self.setp = None
        self.watchdog = None
        
        # Subscribe to joint angles topic
        rospy.Subscriber('/teleoperation/joint_angles', JointState, self.joint_state_callback)
        _log.info("Subscribed to /teleoperation/joint_angles topic")
        
        # Initialize RTDE connection
        self.init_rtde()
        
    def init_rtde(self):
        """Initialize RTDE connection to UR5e robot"""
        try:
            _log.info(f"Connecting to UR5e at {ROBOT_HOST}:{ROBOT_PORT}...")
            self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
            self.con.connect()
            
            # Get controller version
            version = self.con.get_controller_version()
            _log.info(f"UR Controller version: {version[0]}.{version[1]}.{version[2]}")
            
            # Load configuration
            if not os.path.exists(CONFIG_FILE):
                _log.error(f"Config file not found: {CONFIG_FILE}")
                return False
                
            conf = rtde_config.ConfigFile(CONFIG_FILE)
            state_names, state_types = conf.get_recipe("state")
            setp_names, setp_types = conf.get_recipe("setp")
            watchdog_names, watchdog_types = conf.get_recipe("watchdog")
            
            # Setup recipes
            self.con.send_output_setup(state_names, state_types)
            self.setp = self.con.send_input_setup(setp_names, setp_types)
            self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)
            
            # Initialize setpoint to zero
            for i in range(6):
                self.setp.__dict__[f"input_double_register_{i}"] = 0.0
            
            # Initialize watchdog
            self.watchdog.input_int_register_0 = 0
            
            # Start data synchronization
            if not self.con.send_start():
                _log.error("Failed to start RTDE synchronization")
                return False
            
            self.rtde_connected = True
            _log.info("RTDE connection established successfully!")
            
            # Start watchdog thread
            rospy.Timer(rospy.Duration(1.0), self.watchdog_callback)
            
            return True
            
        except Exception as e:
            _log.error(f"Failed to initialize RTDE: {e}")
            return False
    
    def watchdog_callback(self, event):
        """Periodically send watchdog to keep connection alive"""
        if self.rtde_connected and self.watchdog is not None:
            self.watchdog.input_int_register_0 = 1
            self.con.send(self.watchdog)
    
    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        if not self.rtde_connected:
            return
        
        try:
            # Extract arm joint angles
            arm_joints = []
            for joint_name in ARM_JOINT_NAMES:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    angle_deg = msg.position[idx]
                    # Convert degrees to radians
                    angle_rad = math.radians(angle_deg)
                    arm_joints.append(angle_rad)
                else:
                    _log.warn(f"Joint {joint_name} not found in message")
                    return
            
            # Send joint angles to robot via RTDE
            for i in range(6):
                self.setp.__dict__[f"input_double_register_{i}"] = arm_joints[i]
            
            self.con.send(self.setp)
            rospy.logdebug(f"Sent joint angles: {[math.degrees(a) for a in arm_joints]}")
            
        except Exception as e:
            _log.error(f"Error processing joint state: {e}")
    
    def shutdown(self):
        """Cleanup on shutdown"""
        if self.con is not None:
            try:
                self.con.send_pause()
                self.con.disconnect()
                _log.info("RTDE connection closed")
            except:
                pass


def main():
    try:
        node = UR5eTeleopNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        _log.info("Shutting down...")


if __name__ == '__main__':
    main()

