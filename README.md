# K100 ROS 2 Motion Planning

本项目包含 K100 机器人的运动规划功能，支持 BrainCo 和 RoHand 两种灵巧手配置。

## 1. 编译与环境配置

### 编译工作空间
```bash
colcon build --packages-select k100_motion_planning
# 或者编译所有包
colcon build
```

### 配置环境变量
```bash
# 如果使用 zsh
source install/setup.zsh

# 如果使用 bash
source install/setup.bash
```

## 2. 启动节点

### 步骤 1: 启动 MoveIt 2 和 RViz
根据您的手型配置选择启动命令：

**BrainCo 灵巧手 (默认):**
```bash
ros2 launch k100_brainco_moveit_config demo.launch.py
# 若无需 RViz 可视化界面:
ros2 launch k100_brainco_moveit_config demo.launch.py use_rviz:=false
```

**RoHand 灵巧手:**
```bash
ros2 launch k100_rohand_moveit_config demo.launch.py
# 若无需 RViz 可视化界面:
ros2 launch k100_rohand_moveit_config demo.launch.py use_rviz:=false
```

### 步骤 2: 启动运动规划节点
```bash
# 默认 (BrainCo 手型)
ros2 launch k100_motion_planning k100_motion_planning.launch.py

# 指定 RoHand 手型
ros2 launch k100_motion_planning k100_motion_planning.launch.py hand_name:=k100_rohand
```
> **注意**: 规划节点启动时，关于 `right_hand`、`left_hand` 以及 `other_joints` 的报错可以忽略。

![alt text](<Screenshot from 2025-09-15 18-13-10.png>)

## 3. 服务调用测试

### 3.1 关节空间规划 (PlanJointGoal)

**单臂 (Right Arm):**
```bash
ros2 service call /plan_joint_goal k100_motion_planning/srv/PlanJointGoal "{group_name: 'right_arm', joint_angles: [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]}"
```

**双臂 (Both Arms):**
> 注意：关节数组顺序通常为 [左臂关节..., 右臂关节...]，共 14 个关节。
```bash
ros2 service call /plan_joint_goal k100_motion_planning/srv/PlanJointGoal "{group_name: 'both_arms', joint_angles: [0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 3.2 播放 Rosbag 轨迹 (PlayRosbag)
支持时间参数化优化 (TimeOptimalTrajectoryGeneration)。

```bash
ros2 service call /play_rosbag k100_motion_planning/srv/PlayRosbag "{group_name: 'left_arm', file_path: '/home/x100/wh/k100_ros2/src/k100_motion_planning/bag/rosbag2_2025_12_03-15_49_22/rosbag2_2025_12_03-15_49_22_0.db3', enable_time_parameterization: true}"
```

### 3.3 笛卡尔空间规划 (PlanPoseGoal)
```bash
ros2 service call /plan_pose_goal k100_motion_planning/srv/PlanPoseGoal "{group_name: 'left_arm', pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.22356, y: 0.21, z: 0.87223}, orientation: {x: 0.6145, y: -0.61441, z: 0.34989, w: -0.3498}}}}"
```

### 3.4 逆运动学求解 (ComputeIK)
```bash
ros2 service call /compute_ik k100_motion_planning/srv/ComputeIK "{group_name: 'left_arm', pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.22356, y: 0.21, z: 0.87223}, orientation: {x: 0.6145, y: -0.61441, z: 0.34989, w: -0.3498}}}}"
```