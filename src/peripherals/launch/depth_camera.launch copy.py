import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    compiled = os.environ['need_compile']
    camera_type = os.environ['DEPTH_CAMERA_TYPE']
    namespace = LaunchConfiguration('namespace', default='')
    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    
    depth_camera_name_arg = DeclareLaunchArgument(
        'depth_camera_name',
        default_value='depth_cam'
    )

    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value=''
    )

    app_arg = DeclareLaunchArgument(
        'app',
        default_value='false'
    )

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    launch_args = {
        'camera_name': LaunchConfiguration('depth_camera_name'),
        # 'tf_prefix': namespace,  # 强制让 tf_prefix 跟随 namespace
        'namespace': namespace   # 显式传递 namespace 给子 launch
    }

    if camera_type == 'ascamera':
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/include/ascamera.launch.py')),
            # launch_arguments={
            #     'camera_name': LaunchConfiguration('depth_camera_name'),
            #     'tf_prefix': LaunchConfiguration('tf_prefix'),
            # }.items())
            launch_arguments=launch_args.items())

    elif camera_type == 'aurora':
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/include/aurora930.launch.py')),
            # launch_arguments={
            #     'camera_name': LaunchConfiguration('depth_camera_name'),
            #     'tf_prefix': LaunchConfiguration('tf_prefix'),
            # }.items())
            launch_arguments=launch_args.items())
    else:
        camera_launch = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(peripherals_package_path, 'launch/usb_cam.launch.py')),
                launch_arguments={'namespace': namespace}.items()
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name="depth_cam_base_link",
                arguments=[
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--qx', '-1.5707963267948966',
                    '--qy', '0',
                    '--qz', '-1.5707963267948966',
                    '--qw', '0',
                    '--frame-id', [namespace, '/ascamera_camera_link_0'],
                    '--child-frame-id', [namespace, '/depth_cam_color_frame']
                ]
            ),
        ])

    namespaced_camera_launch = GroupAction([
        PushRosNamespace(namespace),
        camera_launch
    ])

    return LaunchDescription([
        depth_camera_name_arg,
        namespace_arg,
        tf_prefix_arg,
        app_arg,
        # camera_launch,
        namespaced_camera_launch
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
