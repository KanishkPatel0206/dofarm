import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    pkg_name = 'dofarm'
    pkg_share = get_package_share_directory(pkg_name)

    urdf = os.path.join(pkg_share, 'urdf', 'dof_arm.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'dof_arm', '-file', urdf, '-z', '0.0'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/dof_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/dof_arm/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/dof_arm/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        ],      

        remappings=[
            ('/world/empty/model/dof_arm/joint_state', '/joint_states')
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        rviz_node
    ])
