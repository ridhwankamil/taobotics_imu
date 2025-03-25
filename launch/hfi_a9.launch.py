import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='taobotics_imu',  # Replace with your actual package name
            executable='hfi_a9_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},  # Adjust if needed
                {'baudrate': 921600},
                {'gra_normalization': True},
                {'frame_id': "imu"}
            ]
        )
    ])
