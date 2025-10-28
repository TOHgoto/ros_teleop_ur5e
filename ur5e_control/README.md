# UR5e Teleoperation Node

This package provides ROS nodes to control a UR5e robot arm based on joint angles published from Vision Pro teleoperation system.

## Overview

The system subscribes to `/teleoperation/joint_angles` topic which contains:
- 6 arm joints (shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint)
- 20 hand/finger joints

The nodes extract the arm joint angles and send them to the UR5e robot.

## Configuration

**Robot IP**: 192.168.1.101  
**Host IP**: 192.168.1.102

Update the `ROBOT_HOST` variable in the node files if your robot has a different IP address.

## Two Approaches

### 1. URScript Node (Recommended for testing)

File: `ur5e_urscript_node.py`

- Directly sends `movej()` commands via socket (port 30002)
- No pre-loaded program required on robot
- Simpler setup
- Suitable for testing connectivity

### 2. RTDE Node

File: `ur5e_teleop_node.py`

- Uses RTDE protocol (port 30004)
- Requires a control program on the robot (rtde_control_loop.urp)
- Better for real-time control

## Usage

### Testing Connectivity

1. **Start the teleop node**:
   ```bash
   python3 ur5e_urscript_node.py
   ```
   Or with parameters:
   ```bash
   python3 ur5e_urscript_node.py _speed:=0.5 _acceleration:=0.5
   ```

2. **In another terminal, publish test joint angles**:
   ```bash
   python3 test_teleop.py
   ```

3. **Verify**: The robot should move to the test poses.

### With Vision Pro Data

Once the Vision Pro system is running and publishing to `/teleoperation/joint_angles`, simply start the teleop node:

```bash
python3 ur5e_urscript_node.py
```

## Parameters

- `~speed`: Joint speed in rad/s (default: 0.5)
- `~acceleration`: Joint acceleration (default: 0.5)

## Joint Mapping

The system maps the following joints from ROS topic to UR5e:

| ROS Joint Name | UR5e Joint | Notes |
|---------------|-----------|-------|
| shoulder_pan_joint | Base rotation | Range: ±360° |
| shoulder_lift_joint | Shoulder | Range: ±360° |
| elbow_joint | Elbow | Range: ±360° |
| wrist_1_joint | Wrist 1 | Range: ±360° |
| wrist_2_joint | Wrist 2 | Range: ±360° |
| wrist_3_joint | Wrist 3 | Range: ±360° |

## Troubleshooting

### Connection Failed

1. Check if robot IP is correct
2. Ensure robot is powered on
3. Verify network connection between host and robot

### Robot Not Moving

1. Check if robot is in Remote mode (not Local control)
2. Ensure Emergency Stop is released
3. Check ROS topic is publishing: `rostopic echo /teleoperation/joint_angles`
4. Verify robot's protective stop is not active

### Safety Stop

- UR5e has built-in safety features that will stop motion if joints move too fast
- Try lowering the `speed` parameter
- Gradually increase speed once stable operation is confirmed

## Notes

- Joint angles are converted from degrees (ROS) to radians (URScript)
- The robot will try to reach the target position as fast as the speed/acceleration allows
- For smoother operation with Vision Pro, consider using a low-pass filter on the joint angles

