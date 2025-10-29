# ros_teleop_ur5e
Teleoperate a UR5e robot using Vision Pro tracking via ROS

## Overview

This project enables teleoperation of a UR5e robot arm using:
- **Vision Pro** for capturing head, wrist, and hand tracking data
- **ROS Noetic** for communication
- **UR5e robot** controlled via URScript/RTDE protocols

## System Architecture

```
Vision Pro → ROS Topic (/teleoperation/joint_angles) → UR5e Control Node → UR5e Robot
```

### Data Flow

1. Vision Pro captures arm and hand poses
2. Data published to `/teleoperation/joint_angles` topic (JointState message)
3. UR5e control node extracts 6 arm joint angles
4. Joint angles sent to UR5e robot via socket connection
5. Robot moves to match operator's arm pose

## Requirements

- Ubuntu 20.04
- ROS1 Noetic
- Network connection between host (192.168.1.102) and robot (192.168.1.101)

## Quick Start

### 1. Test Connectivity (Simplest)

Terminal 1 - Start the control node:
```bash
cd ur5e_control
python3 ur5e_urscript_node.py
```

Terminal 2 - Publish test pose:
```bash
python3 simple_test_publisher.py
```

The robot should attempt to move to the test pose.

### 2. Full Workflow with Vision Pro

1. Ensure Vision Pro system is running and publishing to `/teleoperation/joint_angles`
2. Start the teleop node:
   ```bash
   python3 ur5e_urscript_node.py
   ```
3. Wear Vision Pro and start arm movement - robot should follow your arm!

## Configuration Files

- `config/ur5e_control_config.xml` - RTDE configuration (for RTDE approach)
- Robot IP: Configured in node files (default: 192.168.1.101)

## Notes

- Joint angles are in degrees from Vision Pro topic
- Converted to radians for URScript
- Speed and acceleration can be adjusted via ROS parameters
- Currently only work on the communication between ROS and the real UR. If remapping is involved, other solutions will be needed.