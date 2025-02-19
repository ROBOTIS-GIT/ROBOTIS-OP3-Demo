import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  op3_manager_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [get_package_share_directory('op3_manager'), '/launch/op3_manager.launch.py']
    )
  )

  usb_cam_node = Node(
    package='usb_cam',
    executable='usb_cam_node_exe',
    name='usb_cam_node_exe',
    output='log',
    parameters=[{
      'video_device': '/dev/video0',
      'image_width': 1280,
      'image_height': 720,
      'framerate': 30.0,
      'camera_frame_id': 'cam_link',
      'camera_name': 'camera',
      'io_method': 'mmap',
      'pixel_format': 'mjpeg2rgb',
      'av_device_format': 'YUV422P',
    }],
    remappings=[('/image_raw', '/usb_cam_node/image_raw')]
  )

  return LaunchDescription([
    op3_manager_launch,
    usb_cam_node
  ])
