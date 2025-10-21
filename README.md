--colcon build 编译

--source install/setup.zsh（若是bash则修改成install/setup.bash）

--ros2 launch k100_rohand_moveit_config demo.launch.py（傲意灵巧手） 或 ros2 launch k100_brainco_moveit_config demo.launch.py （强脑灵巧手）
  启动moveit2和rviz

--若无需可视化可使用以下命令
  ros2 launch k100_rohand_moveit_config demo.launch.py use_rviz:=false 或 ros2 launch k100_brainco_moveit_config demo.launch.py use_rviz:=false

--ros2 launch k100_motion_planning k100_motion_planning.launch.py 启动规划结点（默认 BrainCo 手型）
  ros2 launch k100_motion_planning k100_motion_planning.launch.py hand_name:=k100_rohand   启动规划结点（RoHand 手型）

PS：规划结点启动时，关于right_hand和left_hand以及other_joints的报错可以忽略
![alt text](<Screenshot from 2025-09-15 18-13-10.png>)