from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='configs/racetrack1.yaml', description='scenario file'),
        Node(
            package='racetrack',
            executable='racetrack_node',
            name='racetrack_node',
            parameters=[{
                'scenario': LaunchConfiguration('scenario')
            }]
        ),
#        Node(
#            package='joy_control',
#            executable='joy_control_node',
#            name='joy_control_node'
#        ),
        Node(
            package='quadrotor_sim',
            executable='quadrotor_sim_node',
            name='quadrotor_sim_node'
        ),
        Node(
            package='brain',
            executable='brain_node',
            name='brain_node'
        ),
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_depth_optical_frame']
        ),

        # Static TF: base_link → imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )

    ])
