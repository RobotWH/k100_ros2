from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os

# 获取当前工作空间的源代码路径
current_file_path = os.path.abspath(__file__)
workspace_src = os.path.dirname(os.path.dirname(current_file_path))

def generate_launch_description():

    # 声明轨迹配置文件参数
    trajectory_config_arg = DeclareLaunchArgument(
        'trajectory_config_file',
        default_value=os.path.join(workspace_src, "config", "trajectory_config.yaml"),
        description='Path to trajectory configuration file'
    )

    hand_name_arg = DeclareLaunchArgument(
        'hand_name',
        default_value='k100_brainco',
        description='选择灵巧手配置：k100_brainco 或 k100_rohand'
    )

    trajectory_config = LaunchConfiguration('trajectory_config_file')

    def launch_setup(context):
        hand_name = LaunchConfiguration('hand_name').perform(context)
        supported_hands = {
            'k100_brainco': 'k100_brainco_moveit_config',
            'k100_rohand': 'k100_rohand_moveit_config',
        }

        if hand_name not in supported_hands:
            raise RuntimeError(
                f"hand_name={hand_name} 不受支持，可选值：{', '.join(supported_hands.keys())}"
            )

        moveit_config = MoveItConfigsBuilder(
            robot_name='k100',
            package_name=supported_hands[hand_name]
        ).to_moveit_configs()

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
                {'trajectory_config_file': trajectory_config},
            ]
        )

        return [motion_node]

    return LaunchDescription([
        trajectory_config_arg,
        hand_name_arg,
        OpaqueFunction(function=launch_setup),
    ])
