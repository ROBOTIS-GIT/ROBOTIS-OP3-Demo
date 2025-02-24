from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    
    op3_ball_detector_pkg_path = FindPackageShare('op3_ball_detector')
    camera_param_path = PathJoinSubstitution([op3_ball_detector_pkg_path, 'config', 'camera_param.yaml'])
    usb_cam_node = Node(
            package='usb_cam', 
            namespace='usb_cam_node',
            executable='usb_cam_node_exe', 
            output='screen',
            parameters=[camera_param_path],
    )

    op3_ball_detector_parameter_path = PathJoinSubstitution([op3_ball_detector_pkg_path, 'config', 'ball_detector_params.yaml'])
    
    ball_detector_node = Node(
        package='op3_ball_detector',
        namespace='ball_detector_node',
        executable='ball_detector_node',
        output='screen',
        # remappings=[('/ball_detector_node/image_in', '/image_raw'),
        #             ('/ball_detector_node/cameraInfo_in', '/camera_info'),],
        remappings=[('/ball_detector_node/image_in', '/usb_cam_node/image_raw/compressed'),
                    ('/ball_detector_node/cameraInfo_in', '/usb_cam_node/camera_info'),],
        parameters=[{"yaml_path": op3_ball_detector_parameter_path}, op3_ball_detector_parameter_path]
    )
    
    
    ld.add_action(usb_cam_node)
    ld.add_action(ball_detector_node)

    return ld
