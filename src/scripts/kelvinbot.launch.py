import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    if(len(sys.argv) < 4):
        path_to_package = "."
    else:
        path_to_package = sys.argv[3][10::]
        if path_to_package[0] == "\"":
            path_to_package = path_to_package[1:len(path_to_package)-1:]

    gz_args = DeclareLaunchArgument(
        name='gz_args',
        default_value= path_to_package+'/src/sdf/world.sdf',
        description='World file (.sdf file).'
    )

    gazebo = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(gz_launch_path),
                        launch_arguments={
                            'gz_args': LaunchConfiguration('gz_args')
                        }.items()
                    )

    spawn_robot = Node(package='ros_gz_sim', executable='create',
                        arguments=['-file', path_to_package+"/src/urdf/robot.urdf"],
                        output='screen')

    spawn_gz_ros_bridge =  Node(package='ros_gz_bridge', executable='parameter_bridge',
                                arguments=["/thermal_camera@sensor_msgs/msg/Image@gz.msgs.Image"],
                                output='screen')

    object_detection =  ExecuteProcess(
                            cmd=['python3', path_to_package+'/src/scripts/object_detection.py'],
                            output='screen'
                        )

    rqt_image_view = Node(package='rqt_image_view', executable='rqt_image_view',
                            output='screen')

    return LaunchDescription([
        gz_args,
        gazebo,
        spawn_robot,
        spawn_gz_ros_bridge,
        object_detection,
        rqt_image_view
    ])    