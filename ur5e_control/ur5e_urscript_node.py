#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR5e URScript Node
Subscribes to /teleoperation/joint_angles topic and sends arm joint positions 
to UR5e via URScript on port 30002

This is a simpler approach that directly sends move commands without requiring
a pre-loaded program on the robot.
"""

import sys
import os
import rospy
import math
import logging
import socket
import struct
from sensor_msgs.msg import JointState

# Configure logging
logging.basicConfig(level=logging.INFO)
_log = logging.getLogger("UR5e_URScript")

# Robot configuration
ROBOT_HOST = "192.168.1.101"  # UR5e robot IP
ROBOT_PORT = 30002  # URScript socket port

# Joint names for UR5e arm
ARM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]


class UR5eURScriptNode:
    def __init__(self):
        rospy.init_node('ur5e_urscript_node', anonymous=True)
        _log.info("Initializing UR5e URScript Node...")
        
        # Socket connection
        self.sock = None
        self.connected = False
        self.lock = False  # Prevent multiple simultaneous commands
        
        # Subscribe to joint angles topic
        rospy.Subscriber('/teleoperation/joint_angles', JointState, self.joint_state_callback)
        _log.info("Subscribed to /teleoperation/joint_angles topic")
        
        # Get parameters
        self.speed = rospy.get_param("~speed", 0.5)  # Speed of movement (rad/s)
        self.acceleration = rospy.get_param("~acceleration", 0.5)  # Acceleration
        self.blend_radius = rospy.get_param("~blend_radius", 0.0)  # Blend radius for smoother motion
        
        # Initialize socket connection
        self.init_socket()
        
    def init_socket(self):
        """Initialize socket connection to UR5e robot"""
        try:
            _log.info(f"Connecting to UR5e at {ROBOT_HOST}:{ROBOT_PORT}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((ROBOT_HOST, ROBOT_PORT))
            
            self.connected = True
            _log.info("Socket connection established successfully!")
            
            return True
            
        except Exception as e:
            _log.error(f"Failed to connect to UR5e: {e}")
            self.connected = False
            return False
    
    def send_urscript(self, script):
        """Send URScript command to robot"""
        if not self.connected or self.sock is None:
            return False
        
        try:
            # Prepend script length
            script_with_length = struct.pack('>i', len(script)) + script.encode('utf-8')
            self.sock.send(script_with_length)
            return True
        except Exception as e:
            _log.error(f"Failed to send script: {e}")
            self.connected = False
            return False
    
    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        if not self.connected or self.lock:
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
                    rospy.logwarn(f"Joint {joint_name} not found in message")
                    return
            
            # Create URScript command to move joints
            joint_positions = "[" + ", ".join([f"{j:.6f}" for j in arm_joints]) + "]"
            
            # URScript command to move to joint positions
            # Using servoj for more responsive control (better for teleoperation)
            # t=0.02 means 50Hz update rate
            script = f"""
def move_to_joints():
    servoj({joint_positions}, t=0.02, lookahead_time=0.1, gain=300)
end
move_to_joints()
"""
            
            # Send command
            if self.send_urscript(script):
                rospy.logdebug(f"Sent joint angles: {[math.degrees(a) for a in arm_joints]}")
            
        except Exception as e:
            rospy.logerr(f"Error processing joint state: {e}")
    
    def shutdown(self):
        """Cleanup on shutdown"""
        if self.sock is not None:
            try:
                self.sock.close()
                _log.info("Socket connection closed")
            except:
                pass


def main():
    try:
        node = UR5eURScriptNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        _log.info("Shutting down...")


if __name__ == '__main__':
    main()

