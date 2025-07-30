from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{'video_device': '/dev/video2'}],
            remappings=[('image_raw', '/camera/image_raw')],
        ),
        Node(
            package='figure_detector',
            executable='figure_node',
            name='figure_node',
            output='screen'
        )
    ])
        # 📝 Отладка: можно вручную проверить подключенные камеры командой:
        # v4l2-ctl --list-devices
        #
        # И запустить нужную вручную так:
        # ros2 run usb_cam usb_cam_node_exe --ros-args -r image_raw:=/camera/image_raw -p video_device:=/dev/video1