import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)
from launch_ros.actions import Node

# Set LOG format


# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('leapmotion_wrapper'),
    'config',
    'common.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('leapmotion_wrapper'),
    'urdf',
    'leapmotion_descr.urdf.xacro'
)


def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    arr = str.split(',')

    return arr


def launch_setup(context, *args, **kwargs):
    wrapper_dir = get_package_share_directory('leapmotion_wrapper')

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    sensor_name = LaunchConfiguration('sensor_name')
    sensor_model = LaunchConfiguration('sensor_model')

    node_name = LaunchConfiguration('node_name')

    config_common_path = LaunchConfiguration('config_path')

    leapmotion_id = LaunchConfiguration('leapmotion_id')
    serial_number = LaunchConfiguration('serial_number')

    base_frame = LaunchConfiguration('base_frame')
    sen_pose = LaunchConfiguration('sen_pose')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    sensor_name_val = sensor_name.perform(context)
    sensor_model_val = sensor_model.perform(context)

    if (sensor_name_val == ""):
        sensor_name_val = sensor_model_val

    config_sensor_path = os.path.join(
        get_package_share_directory('leapmotion_wrapper'),
        'config',
        sensor_model_val + '.yaml'
    )

    # Convert 'sen_pose' parameter
    sen_pose_str = sen_pose.perform(context)
    sen_pose_array = parse_array_param(sen_pose_str)

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=sensor_name_val,
        executable='robot_state_publisher',
        name='leapmotion_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', sensor_name_val, ' ',
                    'camera_model:=', sensor_model_val, ' ',
                    'base_frame:=', base_frame, ' ',
                    'sen_pos_x:=', sen_pose_array[0], ' ',
                    'sen_pos_y:=', sen_pose_array[1], ' ',
                    'sen_pos_z:=', sen_pose_array[2], ' ',
                    'sen_roll:=', sen_pose_array[3], ' ',
                    'sen_pitch:=', sen_pose_array[4], ' ',
                    'sen_yaw:=', sen_pose_array[5]
                ])
        }]
    )

    # LeapMotion Wrapper node
    leapmotion_wrapper_node = Node(
        package='leapmotion_wrapper',
        namespace=sensor_name_val,
        executable='leapmotion_wrapper',
        name=node_name,
        output='screen',
        #prefix=['xterm -e valgrind --tools=callgrind'],
        #prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            # YAML files
            config_common_path,  # Common parameters
            config_sensor_path,  # Camera related parameters
            # Overriding
            {
                'general.sensor_name': sensor_name_val,
                'general.sensor_model': sensor_model_val,
                'general.svo_file': svo_path,
                'pos_tracking.base_frame': base_frame,
                'general.leapmotion_id': leapmotion_id,
                'general.serial_number': serial_number,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'pos_tracking.publish_imu_tf': publish_tf
            }
        ]
    )

    return [
        rsp_node,
        leapmotion_wrapper_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
            DeclareLaunchArgument(
                'sensor_name',
                default_value=TextSubstitution(text=""),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`. Leave empty to use the camera model as camera name.'),
            DeclareLaunchArgument(
                'sensor_model',
                description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `leapmotion`.'),
            DeclareLaunchArgument(
                'node_name',
                default_value='zed_node',
                description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`'),
            DeclareLaunchArgument(
                'config_path',
                default_value=TextSubstitution(text=default_config_common),
                description='Path to the YAML configuration file for the camera.'),
            DeclareLaunchArgument(
                'leapmotion_id',
                default_value='0',
                description='The index of the camera to be opened. To be used in multi-camera rigs.'),
            DeclareLaunchArgument(
                'serial_number',
                default_value='0',
                description='The serial number of the camera to be opened. To be used in multi-camera rigs. Has priority with respect to `zed_id`.'),
            DeclareLaunchArgument(
                'publish_urdf',
                default_value='true',
                description='Enable URDF processing and starts Robot State Published to propagate static TF.'),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='true',
                description='Enable publication of the `odom -> base_link` TF.'),
            DeclareLaunchArgument(
                'publish_map_tf',
                default_value='true',
                description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.'),
            DeclareLaunchArgument(
                'xacro_path',
                default_value=TextSubstitution(text=default_xacro_path),
                description='Path to the camera URDF file as a xacro file.'),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text="live"),
                description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.'),
            DeclareLaunchArgument(
                'base_frame',
                default_value='base_link',
                description='Name of the base link.'),
            DeclareLaunchArgument(
                'sen_pose',
                default_value='[0.0,0.0,0.0,0.0,0.0,0.0]',
                description='Pose of the camera with respect to the base frame (i.e. `base_link`): [x,y,z,r,p,y]. Note: Orientation in rad.)'),
            OpaqueFunction(function=launch_setup)
        ]
    )
