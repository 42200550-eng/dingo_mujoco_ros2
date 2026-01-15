import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _clean_urdf_xml(xml: str) -> str:
    xml = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', xml)
    xml = re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)
    xml = re.sub(r'\s+', ' ', xml).strip()
    return xml


def generate_launch_description():
    dingo_config_dir = get_package_share_directory('dingo_config')

    sim_params_file = os.path.join(dingo_config_dir, 'config', 'mujoco_dingo_sim.yaml')
    controller_params_file = os.path.join(dingo_config_dir, 'config', 'dingo_gait_controller_cpp.yaml')

    declare_model = DeclareLaunchArgument(
        'model_xml',
        default_value=os.path.join(dingo_config_dir, 'mujoco', 'dingo_urdf_1to1.xml'),
        description='Path to MuJoCo MJCF model XML',
    )

    declare_cmd_topic = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/joint_group_position_controller/commands',
        description='Joint command topic consumed by mujoco_dingo_sim (Float64MultiArray, 12).',
    )

    declare_enable_gait_controller = DeclareLaunchArgument(
        'enable_gait_controller',
        default_value='true',
        description='Start cmd_vel->gait->IK controller (CHAMP-like) for MuJoCo',
    )

    declare_clock_rate = DeclareLaunchArgument(
        'clock_publish_rate_hz',
        default_value='100.0',
        description='Rate to publish /clock from MuJoCo sim (Hz)',
    )

    declare_render = DeclareLaunchArgument(
        'render',
        default_value='false',
        description='Open MuJoCo viewer window (requires a working DISPLAY/OpenGL)',
    )

    declare_render_rate = DeclareLaunchArgument(
        'render_rate_hz',
        default_value='60.0',
        description='Viewer refresh rate (Hz) when render:=true',
    )

    declare_control_mode = DeclareLaunchArgument(
        'control_mode',
        default_value='ros',
        description="MuJoCo control source: 'ros' (CHAMP commands) or 'viewer' (sliders)",
    )

    declare_initial_positions = DeclareLaunchArgument(
        'initial_positions_csv',
        default_value='0.000999,0.719617,0.000339,0.001002,0.719615,0.000339,0.000999,0.719617,0.000339,0.001002,0.719615,0.000339',
        description='Initial joint positions in mujoco_dingo_sim joint order (12 floats). CSV string.',
    )

    declare_joint_signs = DeclareLaunchArgument(
        'joint_signs_csv',
        default_value='1,1,1, 1,1,1, 1,1,1, 1,1,1',
        description='Per-joint sign map (12 floats). CSV string.',
    )

    # URDF for TF (robot_state_publisher). This does NOT drive physics.
    urdf_xacro_path = os.path.join(dingo_config_dir, 'urdf', 'dingo.urdf.xacro')
    robot_description_content = _clean_urdf_xml(xacro.process_file(urdf_xacro_path).toxml())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    mujoco_sim = Node(
        package='dingo_mujoco',
        executable='mujoco_dingo_sim',
        name='mujoco_dingo_sim',
        output='screen',
        parameters=[
            sim_params_file,
            {
                'use_sim_time': True,
                'model_xml_path': LaunchConfiguration('model_xml'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'clock_publish_rate_hz': ParameterValue(LaunchConfiguration('clock_publish_rate_hz'), value_type=float),
                'control_mode': LaunchConfiguration('control_mode'),
                'render': ParameterValue(LaunchConfiguration('render'), value_type=bool),
                'render_rate_hz': ParameterValue(LaunchConfiguration('render_rate_hz'), value_type=float),
                # Pass as CSV strings so users can paste directly from /mujoco_dingo_sim/dump_pose
                'initial_positions_csv': ParameterValue(LaunchConfiguration('initial_positions_csv'), value_type=str),
                'joint_signs_csv': ParameterValue(LaunchConfiguration('joint_signs_csv'), value_type=str),
            }
        ],
    )

    dingo_gait_controller_cpp = Node(
        package='dingo_gait_controller_cpp',
        executable='dingo_gait_controller_cpp',
        name='dingo_gait_controller_cpp',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gait_controller')),
        parameters=[
            controller_params_file,
            {
                'model_xml_path': LaunchConfiguration('model_xml'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
            }
        ],
    )

    return LaunchDescription([
        declare_model,
        declare_cmd_topic,
        declare_enable_gait_controller,
        declare_clock_rate,
        declare_render,
        declare_render_rate,
        declare_control_mode,
        declare_initial_positions,
        declare_joint_signs,
        robot_state_publisher,
        mujoco_sim,
        dingo_gait_controller_cpp,
    ])
