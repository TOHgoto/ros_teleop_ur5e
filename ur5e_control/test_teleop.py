#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test script to publish test joint angles to /teleoperation/joint_angles topic
This can be used to verify the UR5e teleop connection
"""

import rospy
from sensor_msgs.msg import JointState

def publish_test_joint_angles():
    """Publish test joint angles"""
    rospy.init_node('test_joint_angles_publisher', anonymous=True)
    pub = rospy.Publisher('/teleoperation/joint_angles', JointState, queue_size=10)
    
    rate = rospy.Rate(50)  # 50 Hz
    
    # Test poses in degrees
    test_poses = [
        # Home position
        [-0.0, -70.0, 10.0, 3.8, -57.8, 31.4],
        # Pose 1
        [-30.0, -80.0, 20.0, 10.0, -60.0, 40.0],
        # Pose 2
        [30.0, -60.0, 5.0, -5.0, -50.0, 20.0],
        # Back to home
        [-0.0, -70.0, 10.0, 3.8, -57.8, 31.4],
    ]
    
    pose_idx = 0
    
    # Create joint state message
    msg = JointState()
    msg.name = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint',
        # Add hand joints (not used for arm control but present in the topic)
        'IF', 'IS', 'IP', 'ID',
        'LF', 'LS', 'LP', 'LD',
        'MF', 'MS', 'MP', 'MD',
        'RF', 'RS', 'RP', 'RD',
        'TF', 'TS', 'TP', 'TD'
    ]
    
    # Fill with zeros for all joints
    msg.position = [0.0] * len(msg.name)
    msg.velocity = []
    msg.effort = []
    
    rospy.loginfo("Starting to publish test joint angles...")
    rospy.loginfo("Press Ctrl+C to stop")
    
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        msg.header.frame_id = ''
        
        # Update arm joint positions with current test pose
        arm_pose = test_poses[pose_idx]
        for i in range(6):
            msg.position[i] = arm_pose[i]
        
        # Keep hand joints at zero
        for i in range(6, len(msg.position)):
            msg.position[i] = 0.0
        
        pub.publish(msg)
        
        # Print every 1 second
        if msg.header.seq % 50 == 0:
            rospy.loginfo(f"Publishing pose {pose_idx}: {arm_pose}")
        
        rate.sleep()
        
        # Switch poses every 3 seconds
        if msg.header.seq % 150 == 0:
            pose_idx = (pose_idx + 1) % len(test_poses)
            rospy.loginfo(f"Switching to pose {pose_idx}")


if __name__ == '__main__':
    try:
        publish_test_joint_angles()
    except rospy.ROSInterruptException:
        pass

