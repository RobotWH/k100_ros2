#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class CustomJointStatePublisher(Node):
    def __init__(self):
        super().__init__('custom_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 完整的关节列表 (从 k100_brainco.urdf 提取)
        self.joint_names = [
            'head_yaw_joint',
            'head_pitch_joint',
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
            'left_thumb_proximal_joint',
            'left_thumb_metacarpal_joint',
            'left_index_proximal_joint',
            'left_middle_proximal_joint',
            'left_ring_proximal_joint',
            'left_pinky_proximal_joint',
            'right_thumb_metacarpal_joint',
            'right_thumb_proximal_joint',
            'right_index_proximal_joint',
            'right_middle_proximal_joint',
            'right_ring_proximal_joint',
            'right_pinky_proximal_joint',
        ]
        
        # 初始化位置为 0
        self.joint_positions = [0.0] * len(self.joint_names)
        
        # 可选：设置一些默认姿态，避免碰撞
        # 例如：手臂稍微抬起
        try:
            # self.joint_positions[self.joint_names.index('left_shoulder_roll_joint')] = 1.571
            # self.joint_positions[self.joint_names.index('left_shoulder_yaw_joint')] = -1.571
            # self.joint_positions[self.joint_names.index('left_wrist_pitch_joint')] = -1.571
            # self.joint_positions[self.joint_names.index('right_shoulder_roll_joint')] = 1.571
            # self.joint_positions[self.joint_names.index('right_shoulder_yaw_joint')] = 1.571
            # self.joint_positions[self.joint_names.index('right_wrist_pitch_joint')] = -1.571
            # self.joint_positions[self.joint_names.index('left_pinky_proximal_joint')] = 0.85

            self.joint_positions[self.joint_names.index('left_shoulder_pitch_joint')] = 0.0
            self.joint_positions[self.joint_names.index('left_shoulder_roll_joint')] = -0.0
            self.joint_positions[self.joint_names.index('left_shoulder_yaw_joint')] = -0.0
            self.joint_positions[self.joint_names.index('left_elbow_joint')] = 0.0
            self.joint_positions[self.joint_names.index('left_wrist_roll_joint')] = 0.0
            self.joint_positions[self.joint_names.index('left_wrist_pitch_joint')] = -0.0
            self.joint_positions[self.joint_names.index('left_wrist_yaw_joint')] = 0.0

        except ValueError:
            pass

        self.get_logger().info(f'Publishing joint_states for {len(self.joint_names)} joints')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
