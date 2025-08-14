from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gimbal_control',
            executable='gimbal_control_node',
            name='gimbal_control_node',
            output='screen',
            parameters=[{
                'target_ip': '192.168.144.108',
                'target_port': 2337,
                'local_ip': '192.168.144.111',
                'local_port': 2338,
                'mqtt_broker': 'tcp://47.104.183.127:1983',
                'mqtt_user': 'pxtest',
                'mqtt_password': 'test2025@px',
                'mqtt_sub_topic': 'uavcontrol/gimbal/command/uav1',
                'mqtt_pub_topic': 'uavcontrol/gimbal/state/uav1',
            }]
        )
    ])


