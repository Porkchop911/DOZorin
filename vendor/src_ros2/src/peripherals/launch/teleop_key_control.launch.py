import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare parameter(声明参数)
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=os.environ['HOST'],
        description='Name of the robot'
    )

    # Set ROS naming space(设置ROS命名空间)
    push_namespace = PushRosNamespace(
        namespace=LaunchConfiguration('robot_name')
    )

    # teleop_key_control node(节点)(teleop_key_control节点 node)
    teleop_key_control_node = Node(
        package='peripherals',
        executable='teleop_key_control',
        name='teleop_key_control',
        output='screen',
        prefix='xterm -e'
    )

    # Create launch description(创建启动描述)
    ld = LaunchDescription()

    # Add parameter declaring and ROS naming space setting(添加参数声明和ROS命名空间设置)
    ld.add_action(robot_name_arg)
    ld.add_action(push_namespace)

    # Add a node(添加节点)
    ld.add_action(teleop_key_control_node)

    return ld

if __name__ == '__main__':
    # Create a LaunchDescription object(创建一个LaunchDescription对象)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
