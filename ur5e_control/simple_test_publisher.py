#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple test to publish a single pose to verify connection
"""

import rospy
from sensor_msgs.msg import JointState

def publish_single_pose():
    """Publish a single joint state message"""
    rospy.init_node('simple_test_publisher', anonymous=True)
    pub = rospy.Publisher('/teleoperation/joint_angles', JointState, queue_size=10)
    
    # Wait for subscribers
    rospy.sleep(1.0)
    
    # Create a test pose in degrees
    # Home position with slight variation
    test_pose = [-0.0, -70.0, 10.0, 3.8, -57.8, 31.4]
    
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.header.seq = 1
    msg.header.frame_id = ''
    
    # Arm joint names
    msg.name = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
        # Hand joints (filled with zeros)
        'IF', 'IS', 'IP', 'ID',
        'LF', 'LS', 'LP', 'LD',
        'MF', 'MS', 'MP', 'MD',
        'RF', 'RS', 'RP', 'RD',
        'TF', 'TS', 'TP', 'TD'
    ]
    
    # Fill arm joints with test pose
    msg.position = list(test_pose)
    # Add zeros for hand joints
    msg.position.extend([0.0] * 20)
    
    msg.velocity = []
    msg.effort = []
    
    rospy.loginfo(f"Publishing test pose: {test_pose}")
    rospy.loginfo("This pose will be sent once. Check if robot responds.")
    
    # Publish for a few seconds
    for i in range(10):
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        pub.publish(msg)
        rospy.sleep(0.1)
    
    rospy.loginfo("Test complete. Robot should have moved (or tried to move).")

if __name__ == '__main__':
    try:
        publish_single_pose()
    except rospy.ROSInterruptException:
        pass

