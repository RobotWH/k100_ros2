from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# 简化版：直接用 MoveItConfigsBuilder 生成需要的参数并启动节点

def launch_setup(context, *args, **kwargs):
    hand_name = LaunchConfiguration('hand_name').perform(context)
    moveit_config = MoveItConfigsBuilder(hand_name).to_moveit_configs()
    motion_node = Node(
        package='k100_motion_planning',
        executable='k100_motion_planning_node',
        name='k100_motion_planning_node',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ]
    )
    return [motion_node]

def generate_launch_description():
    hand_name_arg = DeclareLaunchArgument(
        'hand_name',
        default_value='k100_brainco',
        description='hand name for MoveIt configuration'
    )
    return LaunchDescription([
        hand_name_arg,
        OpaqueFunction(function=launch_setup)
    ])
