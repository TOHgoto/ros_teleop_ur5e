# Quick Start Guide - UR5e Teleoperation

## 快速测试连通性

### 步骤 1: 准备机器人

1. 确保UR5e已开机
2. 将机器人切换到 **Remote** 模式（而不是Local）
3. 确保急停按钮已释放
4. 检查网络连接：主机(192.168.1.102) 和 机器人(192.168.1.101) 之间通过网线连接

### 步骤 2: 测试连接

**终端1** - 启动UR5e控制节点：
```bash
cd ur5e_control
python3 ur5e_urscript_node.py
```

如果连接成功，你会看到：
```
INFO - Connecting to UR5e at 192.168.1.101:30002...
INFO - Socket connection established successfully!
INFO - Subscribed to /teleoperation/joint_angles topic
```

**终端2** - 发布测试姿态：
```bash
cd ur5e_control
python3 simple_test_publisher.py
```

### 步骤 3: 观察结果

机器人应该开始移动到测试姿态。如果没有移动：

1. **检查机器人模式**：确保在 Remote 控制模式
2. **检查保护停止**：机器人屏幕上不能有Protective Stop
3. **降低速度**：尝试以更小的速度测试：
   ```bash
   python3 ur5e_urscript_node.py _speed:=0.2
   ```

## 与Vision Pro连接

一旦确认基本连通性工作：

1. 启动Vision Pro系统，确保它正在发布到 `/teleoperation/joint_angles` 话题
2. 运行控制节点：
   ```bash
   python3 ur5e_urscript_node.py
   ```
3. 戴上Vision Pro，移动手臂 - 机器人应该跟随你的移动！

## 参数调整

控制节点支持以下参数：

- `~speed`: 运动速度 (默认: 0.5 rad/s)
- `~acceleration`: 加速度 (默认: 0.5 rad/s²)

示例：
```bash
python3 ur5e_urscript_node.py _speed:=0.3 _acceleration:=0.3
```

较低的值会更安全，较高的值响应更快但可能触发安全保护。

## 问题排查

### 连接失败

```
ERROR - Failed to connect to UR5e: [Errno 111] Connection refused
```

**原因**: 机器人未在监听端口30002  
**解决**: 
- 检查IP地址是否正确
- 确保机器人已开机
- 检查防火墙设置

### 机器人不动

**可能原因**:
1. 机器人处于Local控制模式 → 切换到Remote
2. 有安全停止 → 释放急停，检查安全状态
3. 关节角度超出范围 → 检查发布的角度值是否合理

### 运动不流畅

- 降低 `_speed` 参数
- 检查Vision Pro发布频率是否足够高（推荐50Hz以上）
- 检查网络延迟

## 安全提示

⚠️ **重要**:
- 测试时确保机器人周围无人员
- 将速度参数设低开始（0.1-0.3）
- 保持急停按钮可达
- 观察机器人的运动范围，避免碰撞

## 下一步

成功完成连通性测试后：

1. 与Vision Pro集成测试
2. 调整速度和加速度参数以获得最佳响应
3. 考虑添加滤波以提高运动平滑度

